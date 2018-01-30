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
        char buffer[ 256 ];
        snprintf(buffer, 256, "Attempting to handle function %s (%x)",
                names[function], function);
        LOG_DEBUG( "modbus", buffer );
    }

    handler = ctx->function_handlers[function];
    if( handler == NULL ){
        /* No handler for this function code */
        rsp_length = response_exception(
            ctx, &sft,
            MODBUS_EXCEPTION_ILLEGAL_FUNCTION, rsp, FALSE);
        goto send_response;
    }

    rsp_length = ctx->backend->build_response_basis(&sft, rsp);

    /* The handler will return either the length, or a negative response code */
    ret = handler(ctx->reply_user_ctx, slave, function, req + offset + 1, req_length - 1, rsp, sizeof(rsp) );
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
