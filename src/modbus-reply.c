/*
 * Copyright © 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 *
 * SPDX-License-Identifier: LGPL-2.1+
 *
 * This library implements the Modbus protocol.
 * http://libmodbus.org/
 */
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <config.h>

#include "modbus-private.h"
#include "modbus.h"

static const char *names[] = {
    [MODBUS_FC_READ_COILS] = "read_bits",
    [MODBUS_FC_READ_DISCRETE_INPUTS] = "read_input_bits",
    [MODBUS_FC_READ_HOLDING_REGISTERS] = "read_registers",
    [MODBUS_FC_READ_INPUT_REGISTERS] = "read_input_registers",
    [MODBUS_FC_WRITE_SINGLE_COIL] = "write_bit",
    [MODBUS_FC_WRITE_SINGLE_REGISTER] = "write_register",
    [MODBUS_FC_READ_EXCEPTION_STATUS] = "read_exception_status",
    [MODBUS_FC_WRITE_MULTIPLE_COILS] = "write_multiple_bits",
    [MODBUS_FC_WRITE_MULTIPLE_REGISTERS] = "write_multiple_registers",
    [MODBUS_FC_REPORT_SLAVE_ID] = "report_slave_id",
    [MODBUS_FC_MASK_WRITE_REGISTER] = "mask_write_register",
    [MODBUS_FC_WRITE_AND_READ_REGISTERS] = "write_and_read_registers",
    [MODBUS_FC_WRITE_AND_READ_REGISTERS + 1] = "UNKNOWN",
};

static const char *exceptions[] = {
    [MODBUS_EXCEPTION_ILLEGAL_FUNCTION] = "illegal_function",
    [MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS] = "illegal_data_address",
    [MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE] = "illegal_data_value",
    [MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE] = "slave_or_server_failure",
    [MODBUS_EXCEPTION_ACKNOWLEDGE] = "acknowledge",
    [MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY] = "slave_or_server_busy",
    [MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE] = "negative_acknowledge",
    [MODBUS_EXCEPTION_MEMORY_PARITY] = "memory_parity",
    [MODBUS_EXCEPTION_NOT_DEFINED] = "not_defined",
    [MODBUS_EXCEPTION_GATEWAY_PATH] = "gateway_path",
    [MODBUS_EXCEPTION_GATEWAY_TARGET] = "gateway_target",
};

/* Build the exception response */
static int response_exception(modbus_t *ctx, sft_t *sft,
                              int exception_code, uint8_t *rsp,
                              unsigned int to_flush)
{
    int rsp_length;

    /* Print debug message */
    {
        char buffer[ 256 ];

        snprintf( buffer, 256, "Sending exception %s to function code %s (0x%x)",
            exceptions[exception_code],
            names[sft->function],
            sft->function);
        LOG_DEBUG( "modbus", buffer);
    }

    /* Flush if required */
    if (to_flush) {
        _sleep_response_timeout(ctx);
        modbus_flush(ctx);
    }

    /* Build exception response */
    sft->function = sft->function + 0x80;
    rsp_length = ctx->backend->build_response_basis(sft, rsp);
    rsp[rsp_length++] = exception_code;

    return rsp_length;
}

static void convert_registers_to_bytes(const uint16_t *regs, int len, uint8_t *dest){
   int i;
   int dest_loc = 0;

   for( i = 0; i < len; i++ ){
       dest[ dest_loc++ ] = regs[ i ] >> 8;
       dest[ dest_loc++ ] = regs[ i ] & 0xFF;
   } 
}

static int convert_bytes_to_registers(const uint8_t *bytes, int bytes_len, uint16_t *dest){
    int pos;
    int dest_pos = 0;

    for( pos = 0; pos < bytes_len; pos += 2 ){
        dest[dest_pos++] = (bytes[pos]) << 8 | bytes[pos+1];
    }

    return dest_pos;
}

static int convert_bytes_to_bits(const uint8_t *bits,
                              int bits_len,
                              uint8_t *rsp)
{
    int shift = 0;
    /* Instead of byte (not allowed in Win32) */
    int one_byte = 0;
    int i;
    int offset = 0;

    for (i = 0; i < bits_len; i++) {
        one_byte |= bits[i] << shift;
        if (shift == 7) {
            /* Byte is full */
            rsp[offset++] = one_byte;
            one_byte = shift = 0;
        } else {
            shift++;
        }
    }

    if (shift != 0)
        rsp[offset++] = one_byte;

    return offset;
}

static int convert_bits_to_bytes( const uint8_t* bits,
                                  int bits_len,
                                  uint8_t* as_bytes){
    int shift = 0;
    int val;
    int pos;
    int bytes_pos = 0;

    as_bytes[0] = 0;
    for( pos = 0; pos < bits_len; pos++ ){
        val = bits[pos] << shift;
        as_bytes[bytes_pos] |= val;
        shift++;
        if( shift > 7 ){
            bytes_pos++;
            shift = 0;
        }
    }

    return bytes_pos;
}

static int has_function_callback(modbus_t *ctx,
                   int function ){
    switch( function ){
    case MODBUS_FC_READ_INPUT_REGISTERS:
        return ctx->function_callbacks.read_input > 0;
    case MODBUS_FC_READ_DISCRETE_INPUTS:
        return ctx->function_callbacks.read_discrete > 0;
    case MODBUS_FC_READ_HOLDING_REGISTERS:
        return ctx->function_callbacks.read_holding > 0;
    case MODBUS_FC_WRITE_SINGLE_COIL:
        return ctx->function_callbacks.write_coil > 0;
    case MODBUS_FC_WRITE_SINGLE_REGISTER:
        return ctx->function_callbacks.write_holding > 0;
    case MODBUS_FC_READ_COILS:
        return ctx->function_callbacks.read_coil > 0;
    }

    return 0;
}

static int do_function_callback(modbus_t *ctx, int slave, int function,
                                const uint8_t *req, int req_len,
                                uint8_t *rsp, uint8_t rsp_len ){
    int address;
    int nb;
    int ret;
    uint8_t output_data_u8[ 255 ];
    uint16_t output_data_u16[ 128 ];

    if( req_len < 4 ){
        return -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    address = (req[0] << 8) + req[1];
    nb = (req[2] << 8) + req[3];

    switch( function ){
    case MODBUS_FC_READ_INPUT_REGISTERS:
        ret = ctx->function_callbacks.read_input(ctx->reply_user_ctx, slave, address, nb, output_data_u16 );
        if( ret < 0 || (ret * 2) > MODBUS_MAX_PDU_LENGTH ) goto bad_stat;
        if( ret != nb ) {
            ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
            goto bad_stat;
        }
        convert_registers_to_bytes( output_data_u16, ret, rsp + 1 );
        ret *= 2;
        rsp[0] = ret;
        ret += 1;
        break;
    case MODBUS_FC_READ_DISCRETE_INPUTS:
        ret = ctx->function_callbacks.read_discrete(ctx->reply_user_ctx, slave, address, nb, output_data_u8 );
        if( ret != nb ) {
            ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
            goto bad_stat;
        }
        ret = convert_bits_to_bytes( output_data_u8, ret, rsp + 1 );
        rsp[0] = ret;
        ret += 1;
        break;
    case MODBUS_FC_READ_HOLDING_REGISTERS:
        ret = ctx->function_callbacks.read_holding(ctx->reply_user_ctx, slave, address, nb, output_data_u16 );
        if( ret < 0 || (ret * 2) > MODBUS_MAX_PDU_LENGTH ) goto bad_stat;
        if( ret != nb ) {
            ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
            goto bad_stat;
        }
        convert_registers_to_bytes( output_data_u16, ret, rsp + 1 );
        ret *= 2;
        rsp[0] = ret;
        ret += 1;
        break;
    case MODBUS_FC_WRITE_SINGLE_COIL:
        {
            int bytes_len = convert_bytes_to_bits(req, nb, output_data_u8);
            ret = ctx->function_callbacks.write_coil(ctx->reply_user_ctx, slave, address, bytes_len, output_data_u8 );
        }
        break;
    case MODBUS_FC_WRITE_SINGLE_REGISTER:
        {
            int regs_len = convert_bytes_to_registers(req + 4, req_len - 4, output_data_u16 );
            ret = ctx->function_callbacks.write_holding(ctx->reply_user_ctx, slave, address, regs_len, output_data_u16);
        }
        break;
    case MODBUS_FC_READ_COILS:
        ret = ctx->function_callbacks.read_coil(ctx->reply_user_ctx, slave, address, nb, output_data_u8 );
        if( ret != nb ) {
            ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
            goto bad_stat;
        }
        ret = convert_bits_to_bytes( output_data_u8, ret, rsp + 1 );
        rsp[0] = ret;
        ret += 1;
        break;
    }

bad_stat:
    return ret;
}

/* Send a response to the received request.
   Analyses the request and constructs a response.

   If an error occurs, this function construct the response
   accordingly.
*/
int modbus_reply_callback(modbus_t *ctx, const uint8_t *req, int req_length)
{
    int offset;
    int slave;
    int function;
    uint8_t rsp[MAX_MESSAGE_LENGTH];
    int rsp_length = 0;
    sft_t sft;
    modbus_handle_function handler;
    int ret;

    offset = ctx->backend->header_length;
    slave = req[offset - 1];
    function = req[offset];

    /* special RTU-cases error checking */
    if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
        /* Only accept messages under the following circumstances:
         * - The message is a broadcast address
         * - The message is for us
         * - Our address is MODBUS_SLAVE_ACCEPT_ALL
         */
        int our_address = modbus_get_slave( ctx );
        int accept = FALSE;
        if (slave == MODBUS_BROADCAST_ADDRESS ||
            slave == our_address ||
            our_address == MODBUS_SLAVE_ACCEPT_ALL ){
            accept = TRUE;
        }

        if (!accept){
            char buffer[ 128 ];
            snprintf( buffer, 128, 
                  "slave ID %d is not handled by this instance", slave);
	    LOG_DEBUG( "modbus", buffer );
            return 0;
        }

        // TODO broadcast responses should use the slave-id, probably
    }

    sft.slave = slave;
    sft.function = function;
    sft.t_id = ctx->backend->prepare_response_tid(req, &req_length);

    {
        int names_func;
        if( function <= MODBUS_FC_WRITE_AND_READ_REGISTERS ){
            names_func = function;
        }else{
            names_func = MODBUS_FC_WRITE_AND_READ_REGISTERS + 1;
        }
        char buffer[ 256 ];
        snprintf(buffer, 256, "Attempting to handle function %s (%x)",
                names[names_func], function);
        LOG_DEBUG( "modbus", buffer );
    }

    handler = ctx->function_handlers[function];
    if( handler == NULL ){
        /* No handler for this function code - check specific handlers */
        int valid_function_code = has_function_callback( ctx, function );

        if( !valid_function_code ){
            rsp_length = response_exception(
                ctx, &sft,
                MODBUS_EXCEPTION_ILLEGAL_FUNCTION, rsp, FALSE);
            goto send_response;
        }
    }

    rsp_length = ctx->backend->build_response_basis(&sft, rsp);

    if( has_function_callback( ctx, function ) ){
        ret = do_function_callback(ctx, slave, function,
                                   req + offset + 1,
                                   req_length - 1,
                                   rsp + rsp_length,
                                   sizeof(rsp) - rsp_length );
    }else{
        /* The handler will return either the length, or a negative response code */
        ret = handler(ctx->reply_user_ctx, slave, function,
                                   req + offset + 1,
                                   req_length - 1,
                                   rsp + rsp_length,
                                   sizeof(rsp) - rsp_length );
    }

    if (ret < 0){
        int exception_val = ret * -1;
        rsp_length = response_exception(
            ctx, &sft, exception_val, rsp, TRUE);
        goto send_response;
    }

    rsp_length += ret;

send_response:
    if ((ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU &&
         slave == MODBUS_BROADCAST_ADDRESS) ||
        rsp_length == 0) /* this indicates that the user does not want us to send response,
							probably to trigger a timeout on the other side */
        return 0;
    else
        return _modbus_send_msg(ctx, rsp, rsp_length);
}
