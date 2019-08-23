/*
 * Copyright © 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 *
 * This library implements the Modbus protocol.
 * http://libmodbus.org/
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <limits.h>
#include <time.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif

#include <config.h>

#include "modbus.h"
#include "modbus-private.h"

/* Internal use */
#define MSG_LENGTH_UNDEFINED -1

/* Exported version */
const unsigned int libmodbus_version_major = LIBMODBUS_VERSION_MAJOR;
const unsigned int libmodbus_version_minor = LIBMODBUS_VERSION_MINOR;
const unsigned int libmodbus_version_micro = LIBMODBUS_VERSION_MICRO;

/* Max between RTU and TCP max adu length (so TCP) */
#define MAX_MESSAGE_LENGTH 260

/* Define our log function pointer */
simplelogger_log_function libsynmodbus_log_function = NULL;

/* What kind of processing we are doing */
enum ProcessingType{
    PROCESSING_MASTER,
    PROCESSING_SLAVE
};

const char *modbus_strerror(int errnum) {
    switch (errnum) {
    case EMBXILFUN:
        return "Illegal function";
    case EMBXILADD:
        return "Illegal data address";
    case EMBXILVAL:
        return "Illegal data value";
    case EMBXSFAIL:
        return "Slave device or server failure";
    case EMBXACK:
        return "Acknowledge";
    case EMBXSBUSY:
        return "Slave device or server is busy";
    case EMBXNACK:
        return "Negative acknowledge";
    case EMBXMEMPAR:
        return "Memory parity error";
    case EMBXGPATH:
        return "Gateway path unavailable";
    case EMBXGTAR:
        return "Target device failed to respond";
    case EMBBADCRC:
        return "Invalid CRC";
    case EMBBADDATA:
        return "Invalid data";
    case EMBBADEXC:
        return "Invalid exception code";
    case EMBMDATA:
        return "Too many data";
    case EMBBADSLAVE:
        return "Response not from requested slave";
    default:
        return strerror(errnum);
    }
}

void _error_print(modbus_t *ctx, const char *context)
{
    if (ctx->debug) {
        fprintf(stderr, "ERROR %s", modbus_strerror(errno));
        if (context != NULL) {
            fprintf(stderr, ": %s\n", context);
        } else {
            fprintf(stderr, "\n");
        }
    }
}

void _sleep_response_timeout(modbus_t *ctx)
{
    /* Response timeout is always positive */
#ifdef _WIN32
    /* usleep doesn't exist on Windows */
    Sleep((ctx->response_timeout.tv_sec * 1000) +
          (ctx->response_timeout.tv_usec / 1000));
#else
    /* usleep source code */
    struct timespec request, remaining;
    request.tv_sec = ctx->response_timeout.tv_sec;
    request.tv_nsec = ((long int)ctx->response_timeout.tv_usec) * 1000;
    while (nanosleep(&request, &remaining) == -1 && errno == EINTR) {
        request = remaining;
    }
#endif
}

int modbus_flush(modbus_t *ctx)
{
    int rc;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    rc = ctx->backend->flush(ctx);
    if (rc != -1 && ctx->debug) {
        /* Not all backends are able to return the number of bytes flushed */
        printf("Bytes flushed (%d)\n", rc);
    }
    return rc;
}

/* Computes the length of the expected response */
static unsigned int compute_response_length_from_request(modbus_t *ctx, uint8_t *req)
{
    int length;
    const int offset = ctx->backend->header_length;

    switch (req[offset]) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        /* Header + nb values (code from write_bits) */
        int nb = (req[offset + 3] << 8) | req[offset + 4];
        length = 2 + (nb / 8) + ((nb % 8) ? 1 : 0);
    }
        break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS:
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        /* Header + 2 * nb values */
        length = 2 + 2 * (req[offset + 3] << 8 | req[offset + 4]);
        break;
    case MODBUS_FC_READ_EXCEPTION_STATUS:
        length = 3;
        break;
    case MODBUS_FC_REPORT_SLAVE_ID:
        /* The response is device specific (the header provides the
           length) */
        return MSG_LENGTH_UNDEFINED;
    case MODBUS_FC_MASK_WRITE_REGISTER:
        length = 7;
        break;
    default:
        length = 5;
    }

    return offset + length + ctx->backend->checksum_length;
}

/* Sends a request/response */
int _modbus_send_msg(modbus_t *ctx, uint8_t *msg, int msg_length)
{
    int rc;
    int i;

    msg_length = ctx->backend->send_msg_pre(msg, msg_length);

   {
        char debug_buffer[ 1024 ];
        int bytes = 0;
        int string_len = 0;
        for (i = 0; i < msg_length; i++){
            bytes = snprintf( debug_buffer + string_len, 1024 - string_len, "[%.2X]", msg[i]);
            string_len += bytes;
        }
        LOG_DEBUG( "modbus", debug_buffer );
    }

    /* In recovery mode, the write command will be issued until to be
       successful! Disabled by default. */
    do {
        rc = ctx->backend->send(ctx, msg, msg_length);
        if (rc == -1) {
            _error_print(ctx, NULL);
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
                int saved_errno = errno;

                if ((errno == EBADF || errno == ECONNRESET || errno == EPIPE)) {
                    modbus_close(ctx);
                    _sleep_response_timeout(ctx);
                    modbus_connect(ctx);
                } else {
                    _sleep_response_timeout(ctx);
                    modbus_flush(ctx);
                }
                errno = saved_errno;
            }
        }
    } while ((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) &&
             rc == -1);

    if (rc > 0 && rc != msg_length) {
        errno = EMBBADDATA;
        return -1;
    }

    return rc;
}

int modbus_send_raw_request(modbus_t *ctx, uint8_t *raw_req, int raw_req_length)
{
    sft_t sft;
    uint8_t req[MAX_MESSAGE_LENGTH];
    int req_length;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (raw_req_length < 2 || raw_req_length > (MODBUS_MAX_PDU_LENGTH + 1)) {
        /* The raw request must contain function and slave at least and
           must not be longer than the maximum pdu length plus the slave
           address. */
        errno = EINVAL;
        return -1;
    }

    sft.slave = raw_req[0];
    sft.function = raw_req[1];
    /* The t_id is left to zero */
    sft.t_id = 0;
    /* This response function only set the header so it's convenient here */
    req_length = ctx->backend->build_response_basis(&sft, req);

    if (raw_req_length > 2) {
        /* Copy data after function code */
        memcpy(req + req_length, raw_req + 2, raw_req_length - 2);
        req_length += raw_req_length - 2;
    }

    return _modbus_send_msg(ctx, req, req_length);
}

/*
 *  ---------- Request     Indication ----------
 *  | Client | ---------------------->| Server |
 *  ---------- Confirmation  Response ----------
 */

/* Computes the length to read after the function received */
static uint8_t compute_meta_length_after_function(int function,
                                                  msg_type_t msg_type)
{
    int length;

    if (msg_type == MSG_INDICATION) {
        if (function <= MODBUS_FC_WRITE_SINGLE_REGISTER) {
            length = 4;
        } else if (function == MODBUS_FC_WRITE_MULTIPLE_COILS ||
                   function == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
            length = 5;
        } else if (function == MODBUS_FC_MASK_WRITE_REGISTER) {
            length = 6;
        } else if (function == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
            length = 9;
        } else {
            /* MODBUS_FC_READ_EXCEPTION_STATUS, MODBUS_FC_REPORT_SLAVE_ID */
            length = 0;
        }
    } else {
        /* MSG_CONFIRMATION */
        switch (function) {
        case MODBUS_FC_WRITE_SINGLE_COIL:
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            length = 4;
            break;
        case MODBUS_FC_MASK_WRITE_REGISTER:
            length = 6;
            break;
        default:
            length = 1;
        }
    }

    return length;
}

/* Computes the length to read after the meta information (address, count, etc) */
static int compute_data_length_after_meta(modbus_t *ctx, uint8_t *msg,
                                          msg_type_t msg_type)
{
    int function = msg[ctx->backend->header_length];
    int length;

    if (msg_type == MSG_INDICATION) {
        switch (function) {
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            length = msg[ctx->backend->header_length + 5];
            break;
        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
            length = msg[ctx->backend->header_length + 9];
            break;
        default:
            length = 0;
        }
    } else {
        /* MSG_CONFIRMATION */
        if (function <= MODBUS_FC_READ_INPUT_REGISTERS ||
            function == MODBUS_FC_REPORT_SLAVE_ID ||
            function == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
            length = msg[ctx->backend->header_length + 1];
        } else {
            length = 0;
        }
    }

    length += ctx->backend->checksum_length;

    return length;
}


/* Waits a response from a modbus server or a request from a modbus client.
   This function blocks if there is no replies (3 timeouts).

   The function shall return the number of received characters and the received
   message in an array of uint8_t if successful. Otherwise it shall return -1
   and errno is set to one of the values defined below:
   - ECONNRESET
   - EMBBADDATA
   - EMBUNKEXC
   - ETIMEDOUT
   - read() or recv() error codes
*/

int _modbus_receive_msg(modbus_t *ctx, uint8_t *msg, msg_type_t msg_type)
{
    int rc;
    fd_set rset;
    struct timeval tv;
    struct timeval *p_tv;
    int length_to_read;
    int msg_length = 0;
    _step_t step;

    if (msg_type == MSG_INDICATION) {
        LOG_DEBUG( "modbus", "Waiting for a indication...");
    } else {
        LOG_DEBUG( "modbus", "Waiting for a confirmation...");
    }

    /* Add a file descriptor to the set */
    FD_ZERO(&rset);
    FD_SET(ctx->s, &rset);

    /* We need to analyse the message step by step.  At the first step, we want
     * to reach the function code because all packets contain this
     * information. */
    step = _STEP_FUNCTION;
    length_to_read = ctx->backend->header_length + 1;

    if (msg_type == MSG_INDICATION) {
        /* Wait for a message, we don't know when the message will be
         * received */
        if (ctx->indication_timeout.tv_sec == 0 && ctx->indication_timeout.tv_usec == 0) {
            /* By default, the indication timeout isn't set */
            p_tv = NULL;
        } else {
            /* Wait for an indication (name of a received request by a server, see schema) */
            tv.tv_sec = ctx->indication_timeout.tv_sec;
            tv.tv_usec = ctx->indication_timeout.tv_usec;
            p_tv = &tv;
        }
    } else {
        tv.tv_sec = ctx->response_timeout.tv_sec;
        tv.tv_usec = ctx->response_timeout.tv_usec;
        p_tv = &tv;
    }

    while (length_to_read != 0) {
        rc = ctx->backend->select(ctx, &rset, p_tv, length_to_read);
        if (rc == -1) {
            _error_print(ctx, "select");
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
                int saved_errno = errno;

                if (errno == ETIMEDOUT) {
                    _sleep_response_timeout(ctx);
                    modbus_flush(ctx);
                } else if (errno == EBADF) {
                    modbus_close(ctx);
                    modbus_connect(ctx);
                }
                errno = saved_errno;
            }
            return -1;
        }

        rc = ctx->backend->recv(ctx, msg + msg_length, length_to_read);
        if (rc == 0) {
            errno = ECONNRESET;
            rc = -1;
        }

        if (rc == -1) {
            _error_print(ctx, "read");
            if ((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) &&
                (errno == ECONNRESET || errno == ECONNREFUSED ||
                 errno == EBADF)) {
                int saved_errno = errno;
                modbus_close(ctx);
                modbus_connect(ctx);
                /* Could be removed by previous calls */
                errno = saved_errno;
            }
            return -1;
        }

        /* Display the hex code of each character received */
        if (ctx->debug) {
            int i;
            for (i=0; i < rc; i++)
                printf("<%.2X>", msg[msg_length + i]);
            fflush(stdout);
        }

        /* Sums bytes received */
        msg_length += rc;
        /* Computes remaining bytes */
        length_to_read -= rc;

        if (length_to_read == 0) {
            switch (step) {
            case _STEP_FUNCTION:
                /* Function code position */
                length_to_read = compute_meta_length_after_function(
                    msg[ctx->backend->header_length],
                    msg_type);
                if (length_to_read != 0) {
                    step = _STEP_META;
                    break;
                } /* else switches straight to the next step */
            case _STEP_META:
                length_to_read = compute_data_length_after_meta(
                    ctx, msg, msg_type);
                if ((msg_length + length_to_read) > (int)ctx->backend->max_adu_length) {
                    errno = EMBBADDATA;
                    _error_print(ctx, "too many data");
                    return -1;
                }
                step = _STEP_DATA;
                break;
            default:
                break;
            }
        }

        if (length_to_read > 0 &&
            (ctx->byte_timeout.tv_sec > 0 || ctx->byte_timeout.tv_usec > 0)) {
            /* If there is no character in the buffer, the allowed timeout
               interval between two consecutive bytes is defined by
               byte_timeout */
            tv.tv_sec = ctx->byte_timeout.tv_sec;
            tv.tv_usec = ctx->byte_timeout.tv_usec;
            p_tv = &tv;
        }
        /* else timeout isn't set again, the full response must be read before
           expiration of response timeout (for CONFIRMATION only) */
    }


    return ctx->backend->check_integrity(ctx, msg, msg_length);
}

/* Receive the request from a modbus master */
int modbus_receive(modbus_t *ctx, uint8_t *req)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->backend->receive(ctx, req);
}

/* Receives the confirmation.

   The function shall store the read response in rsp and return the number of
   values (bits or words). Otherwise, its shall return -1 and errno is set.

   The function doesn't check the confirmation is the expected response to the
   initial request.
*/
int modbus_receive_confirmation(modbus_t *ctx, uint8_t *rsp)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
}

static int check_confirmation(modbus_t *ctx, uint8_t *req,
                              uint8_t *rsp, int rsp_length, 
                              int* error_code )
{
    int rc;
    int rsp_length_computed;
    const int offset = ctx->backend->header_length;
    const int function = rsp[offset];

    if (ctx->backend->pre_check_confirmation) {
        rc = ctx->backend->pre_check_confirmation(ctx, req, rsp, rsp_length);
        if (rc == -1) {
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
                _sleep_response_timeout(ctx);
                modbus_flush(ctx);
            }
            return -1;
        }
    }

    rsp_length_computed = compute_response_length_from_request(ctx, req);

    /* Exception code */
    if (function >= 0x80) {
        if (rsp_length == (offset + 2 + (int)ctx->backend->checksum_length) &&
            req[offset] == (rsp[offset] - 0x80)) {
            /* Valid exception code received */

            int exception_code = rsp[offset + 1];
            if (exception_code < MODBUS_EXCEPTION_MAX) {
                errno = MODBUS_ENOBASE + exception_code;
            } else {
                errno = EMBBADEXC;
            }
            if( error_code != NULL ){
                *error_code = exception_code;
            }
            _error_print(ctx, NULL);
            return -1;
        } else {
            errno = EMBBADEXC;
            _error_print(ctx, NULL);
            return -1;
        }
    }

    /* Check length */
    if ((rsp_length == rsp_length_computed ||
         rsp_length_computed == MSG_LENGTH_UNDEFINED) &&
        function < 0x80) {
        int req_nb_value;
        int rsp_nb_value;

        /* Check function code */
        if (function != req[offset]) {
            {
                char message_buffer[ 1024 ];
                snprintf(message_buffer, 1024,
                        "Received function not corresponding to the request (0x%X != 0x%X)\n",
                        function, req[offset]);
                LOG_ERROR( "modbus", message_buffer );
            }
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
                _sleep_response_timeout(ctx);
                modbus_flush(ctx);
            }
            errno = EMBBADDATA;
            return -1;
        }

        /* Check the number of values is corresponding to the request */
        switch (function) {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS:
            /* Read functions, 8 values in a byte (nb
             * of values in the request and byte count in
             * the response. */
            req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            req_nb_value = (req_nb_value / 8) + ((req_nb_value % 8) ? 1 : 0);
            rsp_nb_value = rsp[offset + 1];
            break;
        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS:
            /* Read functions 1 value = 2 bytes */
            req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            rsp_nb_value = (rsp[offset + 1] / 2);
            break;
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            /* N Write functions */
            req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            rsp_nb_value = (rsp[offset + 3] << 8) | rsp[offset + 4];
            break;
        case MODBUS_FC_REPORT_SLAVE_ID:
            /* Report slave ID (bytes received) */
            req_nb_value = rsp_nb_value = rsp[offset + 1];
            break;
        default:
            /* 1 Write functions & others */
            req_nb_value = rsp_nb_value = 1;
        }

        if (req_nb_value == rsp_nb_value) {
            rc = rsp_nb_value;
        } else {
            {
                char message_buffer[ 1024 ];
                snprintf(message_buffer, 1024,
                        "Quantity not corresponding to the request (%d != %d)\n",
                        rsp_nb_value, req_nb_value);
                LOG_ERROR( "modbus", message_buffer );
            }

            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
                _sleep_response_timeout(ctx);
                modbus_flush(ctx);
            }

            errno = EMBBADDATA;
            rc = -1;
        }
    } else {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Message length not corresponding to the computed length (%d != %d)\n",
                    rsp_length, rsp_length_computed);
            LOG_ERROR( "modbus", message_buffer );
        }
        if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
            _sleep_response_timeout(ctx);
            modbus_flush(ctx);
        }
        errno = EMBBADDATA;
        rc = -1;
    }

    return rc;
}

int modbus_reply_exception(modbus_t *ctx, const uint8_t *req,
                           unsigned int exception_code)
{
    int offset;
    int slave;
    int function;
    uint8_t rsp[MAX_MESSAGE_LENGTH];
    int rsp_length;
    int dummy_length = 99;
    sft_t sft;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    offset = ctx->backend->header_length;
    slave = req[offset - 1];
    function = req[offset];

    sft.slave = slave;
    sft.function = function + 0x80;
    sft.t_id = ctx->backend->prepare_response_tid(req, &dummy_length);
    rsp_length = ctx->backend->build_response_basis(&sft, rsp);

    /* Positive exception code */
    if (exception_code < MODBUS_EXCEPTION_MAX) {
        rsp[rsp_length++] = exception_code;
        return _modbus_send_msg(ctx, rsp, rsp_length);
    } else {
        errno = EINVAL;
        return -1;
    }
}

/* Reads IO status */
static int read_io_status(modbus_t *ctx, int function,
                          int addr, int nb, uint8_t *dest)
{
    int rc;
    int req_length;

    uint8_t req[_MIN_REQ_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);

    rc = _modbus_send_msg(ctx, req, req_length);
    if (rc > 0) {
        int i, temp, bit;
        int pos = 0;
        int offset;
        int offset_end;

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc, NULL);
        if (rc == -1)
            return -1;

        offset = ctx->backend->header_length + 2;
        offset_end = offset + rc;
        for (i = offset; i < offset_end; i++) {
            /* Shift reg hi_byte to temp */
            temp = rsp[i];

            for (bit = 0x01; (bit & 0xff) && (pos < nb);) {
                dest[pos++] = (temp & bit) ? TRUE : FALSE;
                bit = bit << 1;
            }

        }
    }

    return rc;
}

/* Reads the boolean status of bits and sets the array elements
   in the destination to TRUE or FALSE (single bits). */
int modbus_read_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest)
{
    int rc;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_READ_BITS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Too many bits requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_BITS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    rc = read_io_status(ctx, MODBUS_FC_READ_COILS, addr, nb, dest);

    if (rc == -1)
        return -1;
    else
        return nb;
}


/* Same as modbus_read_bits but reads the remote device input table */
int modbus_read_input_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest)
{
    int rc;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_READ_BITS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Too many discrete inputs requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_BITS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    rc = read_io_status(ctx, MODBUS_FC_READ_DISCRETE_INPUTS, addr, nb, dest);

    if (rc == -1)
        return -1;
    else
        return nb;
}

/* Reads the data from a remove device and put that data into an array */
static int read_registers(modbus_t *ctx, int function, int addr, int nb,
                          uint16_t *dest)
{
    int rc;
    int req_length;
    uint8_t req[_MIN_REQ_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Too many registers requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_REGISTERS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);

    rc = _modbus_send_msg(ctx, req, req_length);
    if (rc > 0) {
        int offset;
        int i;

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc, NULL);
        if (rc == -1)
            return -1;

        offset = ctx->backend->header_length;

        for (i = 0; i < rc; i++) {
            /* shift reg hi_byte to temp OR with lo_byte */
            dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
                rsp[offset + 3 + (i << 1)];
        }
    }

    return rc;
}

/* Reads the holding registers of remote device and put the data into an
   array */
int modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest)
{
    int status;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Too many registers requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_REGISTERS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    status = read_registers(ctx, MODBUS_FC_READ_HOLDING_REGISTERS,
                            addr, nb, dest);
    return status;
}

/* Reads the input registers of remote device and put the data into an array */
int modbus_read_input_registers(modbus_t *ctx, int addr, int nb,
                                uint16_t *dest)
{
    int status;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        char message_buffer[ 1024 ];
        snprintf(message_buffer, 1024,
                "Too many input registers requested (%d > %d)\n",
                nb, MODBUS_MAX_READ_REGISTERS);
        LOG_ERROR( "modbus", message_buffer );
        errno = EMBMDATA;
        return -1;
    }

    status = read_registers(ctx, MODBUS_FC_READ_INPUT_REGISTERS,
                            addr, nb, dest);

    return status;
}

/* Write a value to the specified register of the remote device.
   Used by write_bit and write_register */
static int write_single(modbus_t *ctx, int function, int addr, int value)
{
    int rc;
    int req_length;
    uint8_t req[_MIN_REQ_LENGTH];

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx, function, addr, value, req);

    rc = _modbus_send_msg(ctx, req, req_length);
    if (rc > 0) {
        /* Used by write_bit and write_register */
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc, NULL);
    }

    return rc;
}

/* Turns ON or OFF a single bit of the remote device */
int modbus_write_bit(modbus_t *ctx, int addr, int status)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return write_single(ctx, MODBUS_FC_WRITE_SINGLE_COIL, addr,
                        status ? 0xFF00 : 0);
}

/* Writes a value in one register of the remote device */
int modbus_write_register(modbus_t *ctx, int addr, int value)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return write_single(ctx, MODBUS_FC_WRITE_SINGLE_REGISTER, addr, value);
}

/* Write the bits of the array in the remote device */
int modbus_write_bits(modbus_t *ctx, int addr, int nb, const uint8_t *src)
{
    int rc;
    int i;
    int byte_count;
    int req_length;
    int bit_check = 0;
    int pos = 0;
    uint8_t req[MAX_MESSAGE_LENGTH];

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_WRITE_BITS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Writing too many bits (%d > %d)\n",
                    nb, MODBUS_MAX_WRITE_BITS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx,
                                                   MODBUS_FC_WRITE_MULTIPLE_COILS,
                                                   addr, nb, req);
    byte_count = (nb / 8) + ((nb % 8) ? 1 : 0);
    req[req_length++] = byte_count;

    for (i = 0; i < byte_count; i++) {
        int bit;

        bit = 0x01;
        req[req_length] = 0;

        while ((bit & 0xFF) && (bit_check++ < nb)) {
            if (src[pos++])
                req[req_length] |= bit;
            else
                req[req_length] &=~ bit;

            bit = bit << 1;
        }
        req_length++;
    }

    rc = _modbus_send_msg(ctx, req, req_length);
    if (rc > 0) {
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc, NULL);
    }


    return rc;
}

/* Write the values from the array to the registers of the remote device */
int modbus_write_registers(modbus_t *ctx, int addr, int nb, const uint16_t *src)
{
    int rc;
    int i;
    int req_length;
    int byte_count;
    uint8_t req[MAX_MESSAGE_LENGTH];

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_WRITE_REGISTERS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Trying to write to too many registers (%d > %d)\n",
                    nb, MODBUS_MAX_WRITE_REGISTERS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx,
                                                   MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                                   addr, nb, req);
    byte_count = nb * 2;
    req[req_length++] = byte_count;

    for (i = 0; i < nb; i++) {
        req[req_length++] = src[i] >> 8;
        req[req_length++] = src[i] & 0x00FF;
    }

    rc = _modbus_send_msg(ctx, req, req_length);
    if (rc > 0) {
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc, NULL);
    }

    return rc;
}

int modbus_mask_write_register(modbus_t *ctx, int addr, uint16_t and_mask, uint16_t or_mask)
{
    int rc;
    int req_length;
    /* The request length can not exceed _MIN_REQ_LENGTH - 2 and 4 bytes to
     * store the masks. The ugly substraction is there to remove the 'nb' value
     * (2 bytes) which is not used. */
    uint8_t req[_MIN_REQ_LENGTH + 2];

    req_length = ctx->backend->build_request_basis(ctx,
                                                   MODBUS_FC_MASK_WRITE_REGISTER,
                                                   addr, 0, req);

    /* HACKISH, count is not used */
    req_length -= 2;

    req[req_length++] = and_mask >> 8;
    req[req_length++] = and_mask & 0x00ff;
    req[req_length++] = or_mask >> 8;
    req[req_length++] = or_mask & 0x00ff;

    rc = _modbus_send_msg(ctx, req, req_length);
    if (rc > 0) {
        /* Used by write_bit and write_register */
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc, NULL);
    }

    return rc;
}

/* Write multiple registers from src array to remote device and read multiple
   registers from remote device to dest array. */
int modbus_write_and_read_registers(modbus_t *ctx,
                                    int write_addr, int write_nb,
                                    const uint16_t *src,
                                    int read_addr, int read_nb,
                                    uint16_t *dest)

{
    int rc;
    int req_length;
    int i;
    int byte_count;
    uint8_t req[MAX_MESSAGE_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (write_nb > MODBUS_MAX_WR_WRITE_REGISTERS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Too many registers to write (%d > %d)\n",
                    write_nb, MODBUS_MAX_WR_WRITE_REGISTERS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    if (read_nb > MODBUS_MAX_WR_READ_REGISTERS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Too many registers requested (%d > %d)\n",
                    read_nb, MODBUS_MAX_WR_READ_REGISTERS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }
    req_length = ctx->backend->build_request_basis(ctx,
                                                   MODBUS_FC_WRITE_AND_READ_REGISTERS,
                                                   read_addr, read_nb, req);

    req[req_length++] = write_addr >> 8;
    req[req_length++] = write_addr & 0x00ff;
    req[req_length++] = write_nb >> 8;
    req[req_length++] = write_nb & 0x00ff;
    byte_count = write_nb * 2;
    req[req_length++] = byte_count;

    for (i = 0; i < write_nb; i++) {
        req[req_length++] = src[i] >> 8;
        req[req_length++] = src[i] & 0x00FF;
    }

    rc = _modbus_send_msg(ctx, req, req_length);
    if (rc > 0) {
        int offset;

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc, NULL);
        if (rc == -1)
            return -1;

        offset = ctx->backend->header_length;
        for (i = 0; i < rc; i++) {
            /* shift reg hi_byte to temp OR with lo_byte */
            dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
                rsp[offset + 3 + (i << 1)];
        }
    }

    return rc;
}

/* Send a request to get the slave ID of the device (only available in serial
   communication). */
int modbus_report_slave_id(modbus_t *ctx, int max_dest, uint8_t *dest)
{
    int rc;
    int req_length;
    uint8_t req[_MIN_REQ_LENGTH];

    if (ctx == NULL || max_dest <= 0) {
        errno = EINVAL;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx, MODBUS_FC_REPORT_SLAVE_ID,
                                                   0, 0, req);

    /* HACKISH, addr and count are not used */
    req_length -= 4;

    rc = _modbus_send_msg(ctx, req, req_length);
    if (rc > 0) {
        int i;
        int offset;
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc, NULL);
        if (rc == -1)
            return -1;

        offset = ctx->backend->header_length + 2;

        /* Byte count, slave id, run indicator status and
           additional data. Truncate copy to max_dest. */
        for (i=0; i < rc && i < max_dest; i++) {
            dest[i] = rsp[offset + i];
        }
    }

    return rc;
}

void _modbus_init_common(modbus_t *ctx)
{
    /* Slave and socket are initialized to -1 */
    ctx->slave = MODBUS_SLAVE_INIT;
    ctx->s = -1;

    ctx->debug = FALSE;
    ctx->error_recovery = MODBUS_ERROR_RECOVERY_NONE;

    ctx->response_timeout.tv_sec = 0;
    ctx->response_timeout.tv_usec = _RESPONSE_TIMEOUT;

    ctx->byte_timeout.tv_sec = 0;
    ctx->byte_timeout.tv_usec = _BYTE_TIMEOUT;

    ctx->indication_timeout.tv_sec = 0;
    ctx->indication_timeout.tv_usec = 0;
    memset( &(ctx->async_data), 0, sizeof( struct _modbus_async_data ) );
}

/* Define the slave number */
int modbus_set_slave(modbus_t *ctx, int slave)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->backend->set_slave(ctx, slave);
}

int modbus_get_slave(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->slave;
}

int modbus_set_error_recovery(modbus_t *ctx,
                              modbus_error_recovery_mode error_recovery)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    /* The type of modbus_error_recovery_mode is unsigned enum */
    ctx->error_recovery = (uint8_t) error_recovery;
    return 0;
}

int modbus_set_socket(modbus_t *ctx, int s)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    ctx->s = s;
    return 0;
}

int modbus_get_socket(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->s;
}

/* Get the timeout interval used to wait for a response */
int modbus_get_response_timeout(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    *to_sec = ctx->response_timeout.tv_sec;
    *to_usec = ctx->response_timeout.tv_usec;
    return 0;
}

int modbus_set_response_timeout(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec)
{
    if (ctx == NULL ||
        (to_sec == 0 && to_usec == 0) || to_usec > 999999) {
        errno = EINVAL;
        return -1;
    }

    ctx->response_timeout.tv_sec = to_sec;
    ctx->response_timeout.tv_usec = to_usec;
    return 0;
}

/* Get the timeout interval between two consecutive bytes of a message */
int modbus_get_byte_timeout(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    *to_sec = ctx->byte_timeout.tv_sec;
    *to_usec = ctx->byte_timeout.tv_usec;
    return 0;
}

int modbus_set_byte_timeout(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec)
{
    /* Byte timeout can be disabled when both values are zero */
    if (ctx == NULL || to_usec > 999999) {
        errno = EINVAL;
        return -1;
    }

    ctx->byte_timeout.tv_sec = to_sec;
    ctx->byte_timeout.tv_usec = to_usec;
    return 0;
}

/* Get the timeout interval used by the server to wait for an indication from a client */
int modbus_get_indication_timeout(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    *to_sec = ctx->indication_timeout.tv_sec;
    *to_usec = ctx->indication_timeout.tv_usec;
    return 0;
}

int modbus_set_indication_timeout(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec)
{
    /* Indication timeout can be disabled when both values are zero */
    if (ctx == NULL || to_usec > 999999) {
        errno = EINVAL;
        return -1;
    }

    ctx->indication_timeout.tv_sec = to_sec;
    ctx->indication_timeout.tv_usec = to_usec;
    return 0;
}

int modbus_get_header_length(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->backend->header_length;
}

int modbus_connect(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->backend->connect(ctx);
}

void modbus_close(modbus_t *ctx)
{
    if (ctx == NULL)
        return;

    ctx->backend->close(ctx);
}

void modbus_free(modbus_t *ctx)
{
    if (ctx == NULL)
        return;

    ctx->backend->free(ctx);
}

int modbus_set_debug(modbus_t *ctx, int flag)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    ctx->debug = flag;
    return 0;
}

/* Allocates 4 arrays to store bits, input bits, registers and inputs
   registers. The pointers are stored in modbus_mapping structure.

   The modbus_mapping_new_start_address() function shall return the new allocated
   structure if successful. Otherwise it shall return NULL and set errno to
   ENOMEM. */
modbus_mapping_t* modbus_mapping_new_start_address(
    unsigned int start_bits, unsigned int nb_bits,
    unsigned int start_input_bits, unsigned int nb_input_bits,
    unsigned int start_registers, unsigned int nb_registers,
    unsigned int start_input_registers, unsigned int nb_input_registers)
{
    modbus_mapping_t *mb_mapping;

    mb_mapping = (modbus_mapping_t *)malloc(sizeof(modbus_mapping_t));
    if (mb_mapping == NULL) {
        return NULL;
    }

    /* 0X */
    mb_mapping->nb_bits = nb_bits;
    mb_mapping->start_bits = start_bits;
    if (nb_bits == 0) {
        mb_mapping->tab_bits = NULL;
    } else {
        /* Negative number raises a POSIX error */
        mb_mapping->tab_bits =
            (uint8_t *) malloc(nb_bits * sizeof(uint8_t));
        if (mb_mapping->tab_bits == NULL) {
            free(mb_mapping);
            return NULL;
        }
        memset(mb_mapping->tab_bits, 0, nb_bits * sizeof(uint8_t));
    }

    /* 1X */
    mb_mapping->nb_input_bits = nb_input_bits;
    mb_mapping->start_input_bits = start_input_bits;
    if (nb_input_bits == 0) {
        mb_mapping->tab_input_bits = NULL;
    } else {
        mb_mapping->tab_input_bits =
            (uint8_t *) malloc(nb_input_bits * sizeof(uint8_t));
        if (mb_mapping->tab_input_bits == NULL) {
            free(mb_mapping->tab_bits);
            free(mb_mapping);
            return NULL;
        }
        memset(mb_mapping->tab_input_bits, 0, nb_input_bits * sizeof(uint8_t));
    }

    /* 4X */
    mb_mapping->nb_registers = nb_registers;
    mb_mapping->start_registers = start_registers;
    if (nb_registers == 0) {
        mb_mapping->tab_registers = NULL;
    } else {
        mb_mapping->tab_registers =
            (uint16_t *) malloc(nb_registers * sizeof(uint16_t));
        if (mb_mapping->tab_registers == NULL) {
            free(mb_mapping->tab_input_bits);
            free(mb_mapping->tab_bits);
            free(mb_mapping);
            return NULL;
        }
        memset(mb_mapping->tab_registers, 0, nb_registers * sizeof(uint16_t));
    }

    /* 3X */
    mb_mapping->nb_input_registers = nb_input_registers;
    mb_mapping->start_input_registers = start_input_registers;
    if (nb_input_registers == 0) {
        mb_mapping->tab_input_registers = NULL;
    } else {
        mb_mapping->tab_input_registers =
            (uint16_t *) malloc(nb_input_registers * sizeof(uint16_t));
        if (mb_mapping->tab_input_registers == NULL) {
            free(mb_mapping->tab_registers);
            free(mb_mapping->tab_input_bits);
            free(mb_mapping->tab_bits);
            free(mb_mapping);
            return NULL;
        }
        memset(mb_mapping->tab_input_registers, 0,
               nb_input_registers * sizeof(uint16_t));
    }

    return mb_mapping;
}

modbus_mapping_t* modbus_mapping_new(int nb_bits, int nb_input_bits,
                                     int nb_registers, int nb_input_registers)
{
    return modbus_mapping_new_start_address(
        0, nb_bits, 0, nb_input_bits, 0, nb_registers, 0, nb_input_registers);
}

/* Frees the 4 arrays */
void modbus_mapping_free(modbus_mapping_t *mb_mapping)
{
    if (mb_mapping == NULL) {
        return;
    }

    free(mb_mapping->tab_input_registers);
    free(mb_mapping->tab_registers);
    free(mb_mapping->tab_input_bits);
    free(mb_mapping->tab_bits);
    free(mb_mapping);
}

/*
 * Return 1 if the timer has expired(e.g. no result has come back from a 
 * slave device), or 0 otherwise
 */
static int modbus_timer_has_expired( modbus_t *ctx ){
    struct timeval now;
    struct timeval res;

    gettimeofday( &now, NULL );

    timersub( &now, &ctx->async_data.start_time, &res );

    if( res.tv_usec >= ctx->response_timeout.tv_usec &&
        res.tv_sec >= ctx->response_timeout.tv_sec ){
        return 1;
    }

    return 0;
}

static int read_registers_async(modbus_t *ctx, int function, int addr, int nb,
                                modbus_async_callback_t callback,
                                void* callback_data )
{
    int rc;
    int req_length;
    uint8_t* req = ctx->async_data.request;

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Too many registers requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_REGISTERS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    if( callback == NULL ){
        errno = EINVAL;
        return -1;
    }

    ctx->async_data.in_async_operation = 1;
    ctx->async_data.addr = addr;
    ctx->async_data.nb = nb;
    ctx->async_data.callback = callback;
    ctx->async_data.bit_callback = NULL;
    ctx->async_data.callback_data = callback_data;
    ctx->async_data.function_code = function;
    ctx->async_data.parse_step = _STEP_FUNCTION;
    ctx->async_data.raw_data_offset = 0;
    ctx->async_data.length_to_read = ctx->backend->header_length + 1;
    gettimeofday( &ctx->async_data.start_time, NULL );

    req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);

    rc = _modbus_send_msg(ctx, req, req_length);

    return rc;
}

static int read_io_async(modbus_t *ctx, int function, int addr, int nb,
                                modbus_async_bit_callback_t callback,
                                void* callback_data )
{
    int rc;
    int req_length;
    uint8_t* req = ctx->async_data.request;

    if (nb > MODBUS_MAX_READ_BITS) {
        {
            char message_buffer[ 1024 ];
            snprintf(message_buffer, 1024,
                    "Too many bits requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_BITS);
            LOG_ERROR( "modbus", message_buffer );
        }
        errno = EMBMDATA;
        return -1;
    }

    if( callback == NULL ){
        errno = EINVAL;
        return -1;
    }

    ctx->async_data.in_async_operation = 1;
    ctx->async_data.addr = addr;
    ctx->async_data.nb = nb;
    ctx->async_data.callback = NULL;
    ctx->async_data.bit_callback = callback;
    ctx->async_data.callback_data = callback_data;
    ctx->async_data.function_code = function;
    ctx->async_data.parse_step = _STEP_FUNCTION;
    ctx->async_data.raw_data_offset = 0;
    ctx->async_data.length_to_read = ctx->backend->header_length + 1;
    gettimeofday( &ctx->async_data.start_time, NULL );

    req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);

    rc = _modbus_send_msg(ctx, req, req_length);

    return rc;
}

/**
 * Process the timeout condition for the master, and returns if we need to do more
 * processing.
 */
static int modbus_process_master_needs_processing(modbus_t *ctx){
    int rc;

    if( !ctx->async_data.in_async_operation ){
        return FALSE;
    }

    if( ctx->async_data.in_async_operation &&
        modbus_timer_has_expired( ctx ) ){
        LOG_DEBUG( "modbus", "Timeout expired for read operation from slave" );
        /* Our timeout has expired;  purge everything
           in the OS buffer */
        uint8_t purge_buffer[ 128 ];
        do{
            rc = ctx->backend->recv(ctx, purge_buffer, 128 );
        }while( rc > 0 );
        
        ctx->async_data.in_async_operation = FALSE;

        /* Call the callback with an error_code of -1(timeout) */
        if( ctx->async_data.callback ){
            ctx->async_data.callback( ctx, 
                              ctx->async_data.function_code,
                              ctx->async_data.addr, 
                              0,
                              ctx->async_data.data, 
                              -1, 
                              ctx->async_data.callback_data );
        }else if( ctx->async_data.bit_callback ){
            ctx->async_data.bit_callback( ctx,
                              ctx->async_data.function_code,
                              ctx->async_data.addr,
                              0,
                              (uint8_t*)ctx->async_data.data,
                              -1,
                              ctx->async_data.callback_data );
        }
        
        return FALSE;
    }

    return TRUE;
}

static void modbus_process_master_confirmation(modbus_t *ctx, uint8_t* buffer, int* data_offset, uint16_t* dest ){
    int rc;
    int error_code = 0;
    int offset;
    int i;

    rc = check_confirmation(ctx, ctx->async_data.request, buffer, *data_offset,
                            &error_code );
    if (rc == -1){
        if( error_code != 0 ){
            ctx->async_data.in_async_operation = 0;
	    if( ctx->async_data.callback ){
                ctx->async_data.callback( ctx,
                                  ctx->async_data.function_code,
                                  ctx->async_data.addr,
                                  0,
                                  ctx->async_data.data,
                                  error_code,
                                  ctx->async_data.callback_data );
	    }else if( ctx->async_data.bit_callback ){
                ctx->async_data.bit_callback( ctx,
                                  ctx->async_data.function_code,
                                  ctx->async_data.addr,
                                  0,
                                  (uint8_t*)ctx->async_data.data,
                                  error_code,
                                  ctx->async_data.callback_data );
	    }
        }
        LOG_WARN( "modbus", "check_confirmation was invalid, but error code was 0" );
        return;
    }

    offset = ctx->backend->header_length;

    ctx->async_data.in_async_operation = 0;
    switch( ctx->async_data.function_code ){
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        for (i = 0; i < rc; i++) {
            /* shift reg hi_byte to temp OR with lo_byte */
            dest[i] = (buffer[offset + 2 + (i << 1)] << 8) |
                    buffer[offset + 3 + (i << 1)];
        }
        ctx->async_data.callback( ctx,
                          ctx->async_data.function_code,
                          ctx->async_data.addr,
                          ctx->async_data.nb,
                          ctx->async_data.data,
                          error_code,
                          ctx->async_data.callback_data );
	break;
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS:
	{
            uint8_t* data = (uint8_t*)ctx->async_data.data;
	    int pos = 0;
	    int temp, bit;
	    int offset_end;

            offset = ctx->backend->header_length + 2;
            offset_end = offset + rc;
	    /* libmodbus uses 1 byte per bit when reporting the
	     * data back to the user
	     */
	    for (i = offset; i < offset_end; i++) {
                temp = ctx->async_data.raw_data[i];

                for (bit = 0x01; (bit & 0xff) && (pos < ctx->async_data.nb);) {
                    data[pos++] = (temp & bit) ? TRUE : FALSE;
                    bit = bit << 1;
                }
	    }

            ctx->async_data.bit_callback( ctx,
                          ctx->async_data.function_code,
                          ctx->async_data.addr,
                          ctx->async_data.nb,
                          data,
                          error_code,
                          ctx->async_data.callback_data );
	}
	break;
    }
}

static void modbus_process_data(modbus_t *ctx, enum ProcessingType type){
    int rc;
    int i;
    /* Local variables to make this clearer */
    int* data_offset;
    uint8_t* buffer;
    int* length_to_read;
    _step_t* step;
    uint16_t* dest;

    if (ctx == NULL) {
        errno = EINVAL;
        return;
    }

    if( type == PROCESSING_MASTER ){
        if( !modbus_process_master_needs_processing(ctx) ){
            return;
        }
    }

    data_offset = &(ctx->async_data.raw_data_offset);
    buffer = ctx->async_data.raw_data;
    length_to_read = &(ctx->async_data.length_to_read);
    step = &(ctx->async_data.parse_step);
    dest = ctx->async_data.data;

    while( *step != _STEP_DONE ){ 
        if( type == PROCESSING_SLAVE &&
            *data_offset != 0 &&
            modbus_timer_has_expired(ctx) ){
            /* Before we read data, ensure that our timer from an old request hasn't
             * expired.  If it has, reset our offset location.
             * Also assume that the bytes we are processing right now are the 
             * beginning of the next message(function code).
             */
            *data_offset = 0;
            *step = _STEP_FUNCTION;
            LOG_DEBUG( "modbus", "Request from master timed out(incomplete message?)" );
        }

        if( *data_offset == 0 &&
            type == PROCESSING_SLAVE ){
            /* Set the time of the last to be now so we don't process a timeout */
            gettimeofday( &ctx->async_data.start_time, NULL );
            /* Also ensure that we will actually read some bytes.  We need to read
             * at least the header information
             */
            *length_to_read = ctx->backend->header_length + 1;
        }

        rc = ctx->backend->recv(ctx, buffer + *data_offset, *length_to_read );
        if (rc == -1){
            LOG_TRACE( "modbus", "Error reading from device: check errno" );
            return;
        }else if( rc == 0 ){
            LOG_TRACE( "modbus", "No data received from device!" );
            return;
        }

        if( type == PROCESSING_SLAVE ){
            LOG_TRACE( "modbus", "Resetting start time" );
            gettimeofday( &ctx->async_data.start_time, NULL );
        }

        /* Display the hex code of each character received */
        {
            char debug_buffer[ 1024 ];
            int bytes = 0;
            int string_len = 0;
            for (i = 0; i < rc; i++){
                bytes = snprintf( debug_buffer + string_len, 1024 - string_len, "<%.2X>", buffer[*data_offset + i]);
                string_len += bytes;
            }
            LOG_DEBUG( "modbus", debug_buffer );
        }

        *data_offset += rc;
        *length_to_read -= rc;

        if (*length_to_read == 0) {
            msg_type_t message_type  = type == PROCESSING_MASTER ? MSG_CONFIRMATION : MSG_INDICATION;
            /* Switch to next state when it is time */
            switch (*step) {
            case _STEP_FUNCTION:
                /* Function code position */
                *length_to_read = compute_meta_length_after_function(
                    buffer[ctx->backend->header_length],
                    message_type);
                /* If there is META data, break out to trigger read */
/*
AARON
                if (*length_to_read != 0) {
                    *step = _STEP_META;
                    break;
                } 
*/
/* Else... falls into next case */
                if( *length_to_read == 0 ){ 
                    *step = _STEP_DATA; 
                }else{ 
                    *step = _STEP_META; 
                }
                continue;
            case _STEP_META:
                *length_to_read = compute_data_length_after_meta(
                    ctx, buffer, message_type);
                if ((*data_offset + *length_to_read) > (int)ctx->backend->max_adu_length) {
                    errno = EMBBADDATA;
                    _error_print(ctx, "too many data");
                    return;
                }
                *step = _STEP_DATA;
                if( *length_to_read != 0 ) break;
            case _STEP_DATA:
                *step = _STEP_DONE;
            default:
                break;
            }
        }
    }

    if( *step != _STEP_DONE ){
        return;
    }

    rc = ctx->backend->check_integrity(ctx, buffer, *data_offset );
    if( rc == -1 ){
        LOG_WARN( "modbus", "check_integrity failed" );
        /* Reset our buffer */
        *data_offset = 0;
        *step = _STEP_FUNCTION;
        return;
    }

    /*
     * If we are a master, we need to process the confirmation(response) from the slave.
     * Otherwise, we should process this message as a slave.
     */
    if( type == PROCESSING_MASTER ){
        modbus_process_master_confirmation(ctx, buffer, data_offset, dest);
    }else{
        modbus_reply_callback(ctx, buffer, *data_offset);
        *data_offset = 0;
        *step = _STEP_FUNCTION;
    }
}

void modbus_process_data_master(modbus_t *ctx){
    modbus_process_data( ctx, PROCESSING_MASTER );
}

void modbus_process_data_slave(modbus_t *ctx){
    modbus_process_data( ctx, PROCESSING_SLAVE );
}

int modbus_read_registers_async(modbus_t *ctx, int addr, int nb, 
                                modbus_async_callback_t callback,
                                void* callback_data ){
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if( ctx->async_data.in_async_operation &&
        modbus_timer_has_expired( ctx ) ){
        /* Our timeout has expired; continue on but purge everything
           in our buffer */
        uint8_t buffer[ 128 ];
        int rc;
        do{
            rc = ctx->backend->recv(ctx, buffer, 128 );
        }while( rc > 0 );
    }else if( ctx->async_data.in_async_operation ){
        errno = EALREADY;
        return -1;
    }


    return read_registers_async( ctx, MODBUS_FC_READ_HOLDING_REGISTERS,
                                 addr, nb, callback, callback_data );
}

int modbus_read_input_registers_async(modbus_t *ctx, int addr, int nb, 
                                      modbus_async_callback_t callback,
                                      void* callback_data ){
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if( ctx->async_data.in_async_operation &&
        modbus_timer_has_expired( ctx ) ){
        /* Our timeout has expired; continue on but purge everything
           in our buffer */
        uint8_t buffer[ 128 ];
        int rc;
        do{
            rc = ctx->backend->recv(ctx, buffer, 128 );
        }while( rc > 0 );
    }else if( ctx->async_data.in_async_operation ){
        errno = EALREADY;
        return -1;
    }

    return read_registers_async( ctx, MODBUS_FC_READ_INPUT_REGISTERS,
                                 addr, nb, callback, callback_data );
}

int modbus_read_bits_async(modbus_t *ctx, int addr, int nb, modbus_async_bit_callback_t callback, void* data)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if( ctx->async_data.in_async_operation &&
        modbus_timer_has_expired( ctx ) ){
        /* Our timeout has expired; continue on but purge everything
           in our buffer */
        uint8_t buffer[ 128 ];
        int rc;
        do{
            rc = ctx->backend->recv(ctx, buffer, 128 );
        }while( rc > 0 );
    }else if( ctx->async_data.in_async_operation ){
        errno = EALREADY;
        return -1;
    }

    return read_io_async( ctx, MODBUS_FC_READ_COILS, addr, nb, callback, data );
}

int modbus_read_input_bits_async(modbus_t *ctx, int addr, int nb, modbus_async_bit_callback_t callback, void* data)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if( ctx->async_data.in_async_operation &&
        modbus_timer_has_expired( ctx ) ){
        /* Our timeout has expired; continue on but purge everything
           in our buffer */
        uint8_t buffer[ 128 ];
        int rc;
        do{
            rc = ctx->backend->recv(ctx, buffer, 128 );
        }while( rc > 0 );
    }else if( ctx->async_data.in_async_operation ){
        errno = EALREADY;
        return -1;
    }

    return read_io_async( ctx, MODBUS_FC_READ_DISCRETE_INPUTS, addr, nb, callback, data );
}

void modbus_set_log_function( simplelogger_log_function func ){
    libsynmodbus_log_function = func;
}

int modbus_is_in_async_operation( modbus_t* ctx ){
    return ctx->async_data.in_async_operation;
}


void modbus_set_handler_context(modbus_t *ctx, void *user_ctx){
    ctx->reply_user_ctx = user_ctx;
}

void* modbus_get_handler_context(modbus_t *ctx){
    return ctx->reply_user_ctx;
}

int modbus_set_function_handler(modbus_t *ctx, int function_code,
                                           modbus_handle_function handler){
    if( ctx == NULL ||
        function_code < 0 ||
        function_code > 127 ){
        errno = EINVAL;
        return -1;
    }

    ctx->function_handlers[ function_code ] = handler;
    return 0;
}

int modbus_set_input_handler(modbus_t *ctx,
                             modbus_read_input_register_function rd_handler){
    ctx->function_callbacks.read_input = rd_handler;
    return 0;
}

int modbus_set_coil_handlers(modbus_t *ctx, 
                                        modbus_read_coil_function rd_handler,
                                        modbus_write_coil_function wr_handler){
    ctx->function_callbacks.read_coil = rd_handler;
    ctx->function_callbacks.write_coil = wr_handler;
    return 0;
}

int modbus_set_discrete_input_handler(modbus_t *ctx,
                                        modbus_read_discrete_input_function rd_handler){
    ctx->function_callbacks.read_discrete = rd_handler;
    return 0;
}

int modbus_set_holding_handlers(modbus_t *ctx,
                                           modbus_read_holding_register_function rd_handler,
                                           modbus_write_holding_register_function wr_handler){
    ctx->function_callbacks.read_holding = rd_handler;
    ctx->function_callbacks.write_holding = wr_handler;
    return 0;
}

#ifndef HAVE_STRLCPY
/*
 * Function strlcpy was originally developed by
 * Todd C. Miller <Todd.Miller@courtesan.com> to simplify writing secure code.
 * See ftp://ftp.openbsd.org/pub/OpenBSD/src/lib/libc/string/strlcpy.3
 * for more information.
 *
 * Thank you Ulrich Drepper... not!
 *
 * Copy src to string dest of size dest_size.  At most dest_size-1 characters
 * will be copied.  Always NUL terminates (unless dest_size == 0).  Returns
 * strlen(src); if retval >= dest_size, truncation occurred.
 */
size_t strlcpy(char *dest, const char *src, size_t dest_size)
{
    register char *d = dest;
    register const char *s = src;
    register size_t n = dest_size;

    /* Copy as many bytes as will fit */
    if (n != 0 && --n != 0) {
        do {
            if ((*d++ = *s++) == 0)
                break;
        } while (--n != 0);
    }

    /* Not enough room in dest, add NUL and traverse rest of src */
    if (n == 0) {
        if (dest_size != 0)
            *d = '\0'; /* NUL-terminate dest */
        while (*s++)
            ;
    }

    return (s - src - 1); /* count does not include NUL */
}
#endif
