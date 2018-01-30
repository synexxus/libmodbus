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

#include <config.h>

#include "modbus-private.h"
#include "modbus.h"

static int response_io_status(uint8_t *tab_io_status,
                              int address, int nb,
                              uint8_t *rsp, int offset)
{
    int shift = 0;
    /* Instead of byte (not allowed in Win32) */
    int one_byte = 0;
    int i;

    for (i = address; i < address + nb; i++) {
        one_byte |= tab_io_status[i] << shift;
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

static int modbus_verify_registers(void *user_ctx, int function, const uint8_t *req, int req_len,
                              int *address, int *nb){
    int ret = 0;
    modbus_mapping_t *mb_mapping = user_ctx;
    int is_input = (function == MODBUS_FC_READ_INPUT_REGISTERS);

    if( req_len < 4 ){
        return -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    *address = (req[0] << 8) + req[1];
    *nb = (req[2] << 8) + req[3];

    int nb_registers = is_input ? mb_mapping->nb_input_registers : mb_mapping->nb_registers;
    int start_registers = is_input ? mb_mapping->start_input_registers : mb_mapping->start_registers;
    int mapping_address = *address - start_registers;
    if (mapping_address < 0 || (mapping_address + *nb) > nb_registers)
        ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

    return ret;
}

static int modbus_handle_read_input_holding_regs(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *rsp, int rsp_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int address;
    int nb;
    int ret;
    int i;
    uint16_t *tab_registers = (function == MODBUS_FC_READ_INPUT_REGISTERS) ? mb_mapping->tab_input_registers : mb_mapping->tab_registers;
    int start_registers = (function == MODBUS_FC_READ_INPUT_REGISTERS) ? mb_mapping->start_input_registers : mb_mapping->start_registers;
    int mapping_address;
    int length = 0;

    ret = modbus_verify_registers(user_ctx, function, req, req_len, &address, &nb);
    if( ret < 0 ){
        goto out;
    }

    mapping_address = address - start_registers;
    for (i = address; i < mapping_address + nb; i++) {
        rsp[length++] = tab_registers[i] >> 8;
        rsp[length++] = tab_registers[i] & 0xFF;
    }
    ret = nb;

out:
    return ret;
}

static int modbus_handle_write_holding_regs(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *rsp, int rsp_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int address;
    int nb;
    int ret;
    int i,j;
    int mapping_address;

    ret = modbus_verify_registers(user_ctx, function, req, req_len, &address, &nb);
    if( ret < 0 ){
        goto out;
    }

    mapping_address = address - mb_mapping->start_registers;
    for (i = address - mb_mapping->start_registers, j = 0; i < mapping_address + nb; i++, j+=2) {
        mb_mapping->tab_registers[i] = (req[j] << 8) + req[j + 1];
    }
    ret = 0;

out:
    return ret;
}

static int modbus_verify_bits(void *user_ctx, int function, const uint8_t *req, int req_len,
                              int *address, int *nb){
    int max_nb = MODBUS_MAX_READ_BITS;
    int ret = 0;
    modbus_mapping_t *mb_mapping = user_ctx;

    if( req_len < 4 ){
        return -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    *address = (req[0] << 8) + req[1];
    *nb = (req[2] << 8) + req[3];
    
    if (*nb < 1 || max_nb < *nb) {
        /* Bad data request */
        ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }

    /* Note: we only need to check for DISCRETE_INPUTS, as otherwise we default to the coils
     * which are both read/write, so the same code is used to validate
     */
    int start_bits = (function == MODBUS_FC_READ_DISCRETE_INPUTS) ? mb_mapping->start_input_bits : mb_mapping->start_bits;
    int nb_bits = (function == MODBUS_FC_READ_DISCRETE_INPUTS) ? mb_mapping->nb_input_bits : mb_mapping->nb_bits;
    int mapping_address = *address - start_bits;
    if (mapping_address < 0 || (mapping_address + *nb) > nb_bits){
        ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }

    return ret;
}

static int modbus_handle_write_coil(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *return_data, int return_data_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int address;
    int nb;
    int ret;

    ret = modbus_verify_bits(user_ctx, function, req, req_len, &address, &nb);
    if( ret < 0 ){
        goto out;
    }

    modbus_set_bits_from_bytes(mb_mapping->tab_bits, address, nb, req);

out:
    return ret;
}

static int modbus_handle_read_coil_di(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *rsp, int rsp_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int nb;
    int address;
    int ret;
    uint8_t *tab_bits = (function == MODBUS_FC_READ_DISCRETE_INPUTS) ? mb_mapping->tab_input_bits : mb_mapping->tab_bits;

    ret = modbus_verify_bits( user_ctx, function, req, req_len, &address, &nb );
    if( ret < 0 ){
        goto out;
    }

    ret = response_io_status(tab_bits, address, nb, rsp, 0);

out:
    return ret;
}

int modbus_reply(modbus_t *ctx, const uint8_t *req,
                 int req_length, modbus_mapping_t *mb_mapping)
{
    int ret;
    void* old_context = modbus_get_handler_context(ctx);

    /*
     * The default implementations require the context to be a modbus_mapping_t*.
     * By getting and then setting the handler context, we can interleave multiple types
     * of handling the request(e.g. can use both default implementation and custom 
     * implementations for both function codes)
     */
    modbus_set_handler_context(ctx, mb_mapping);
    modbus_set_function_handler(ctx, MODBUS_FC_READ_COILS, modbus_handle_read_coil_di);
    modbus_set_function_handler(ctx, MODBUS_FC_WRITE_SINGLE_COIL, modbus_handle_write_coil);
    modbus_set_function_handler(ctx, MODBUS_FC_WRITE_MULTIPLE_COILS, modbus_handle_write_coil);
    modbus_set_function_handler(ctx, MODBUS_FC_READ_DISCRETE_INPUTS, modbus_handle_read_coil_di);
    modbus_set_function_handler(ctx, MODBUS_FC_READ_HOLDING_REGISTERS, modbus_handle_read_input_holding_regs);
    modbus_set_function_handler(ctx, MODBUS_FC_READ_INPUT_REGISTERS, modbus_handle_read_input_holding_regs);
    modbus_set_function_handler(ctx, MODBUS_FC_WRITE_SINGLE_REGISTER, modbus_handle_write_holding_regs);

    ret = modbus_reply_callback(ctx, req, req_length);

    modbus_set_handler_context(ctx, old_context);

    return ret;
}
