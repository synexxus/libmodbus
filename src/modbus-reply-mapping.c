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
#include <stdio.h>
#include <string.h>

#include "modbus-private.h"
#include "modbus.h"

static int response_io_status(const uint8_t *tab_io_status,
                              int address, int nb,
                              uint8_t *rsp, int offset)
{
    int shift = 0;
    /* Instead of byte (not allowed in Win32) */
    int one_byte = 0;
    int i;

    for (i = address; i < address + nb; i++) {
        one_byte |= (tab_io_status[i] << shift);
        shift++;
        if (shift == 8) {
            /* Byte is full */
            rsp[offset++] = one_byte;
            one_byte = shift = 0;
        }
    }

    if (shift != 0)
        rsp[offset] = one_byte;

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
    if (*nb == 0){
        ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        goto out;
    }

    int nb_registers = is_input ? mb_mapping->nb_input_registers : mb_mapping->nb_registers;
    int start_registers = is_input ? mb_mapping->start_input_registers : mb_mapping->start_registers;
    int mapping_address = *address - start_registers;
    if (mapping_address < 0 || (mapping_address + *nb) > nb_registers)
        ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

out:
    return ret;
}

static int modbus_verify_single_register(void *user_ctx, const uint8_t *req, int req_len, int *address, int *value ){
    int ret = 0;
    modbus_mapping_t *mb_mapping = user_ctx;

    if( req_len < 4 ){
        return -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    *address = (req[0] << 8) + req[1];
    *value = (req[2] << 8) + req[3];

    int nb_registers = mb_mapping->nb_registers;
    int start_registers = mb_mapping->start_registers;
    int mapping_address = *address - start_registers;
    if (mapping_address < 0 || mapping_address > nb_registers)
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
    int length = 1;

    ret = modbus_verify_registers(user_ctx, function, req, req_len, &address, &nb);
    if( ret < 0 ){
        goto out;
    }

    mapping_address = address - start_registers;
    for (i = mapping_address; i < mapping_address + nb; i++) {
        rsp[length++] = tab_registers[i] >> 8;
        rsp[length++] = tab_registers[i] & 0xFF;
    }
    rsp[0] = nb * 2;
    ret = length;

out:
    return ret;
}

static int modbus_handle_write_multiple_registers(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *rsp, int rsp_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int address;
    int nb;
    int ret;
    int i,j;
    int mapping_address;
    int length = 0;

    ret = modbus_verify_registers(user_ctx, function, req, req_len, &address, &nb);
    if( ret < 0 ){
        goto out;
    }

    mapping_address = address - mb_mapping->start_registers;
    for (i = mapping_address, j = 5; i < mapping_address + nb; i++, j += 2) {
        mb_mapping->tab_registers[i] = (req[j] << 8) | req[j + 1];
    }
    memcpy( rsp, req, 4 );
    ret = 4;

out:
    return ret;
}

static int modbus_handle_write_register(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *rsp, int rsp_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int address;
    int value;
    int ret;
    int mapping_address;

    ret = modbus_verify_single_register(user_ctx, req, req_len, &address, &value);
    if( ret < 0 ){
        goto out;
    }

    mapping_address = address - mb_mapping->start_registers;
    mb_mapping->tab_registers[mapping_address] = value;
    memcpy( rsp, req, 4 );
    ret = 4;

out:
    return ret;
}

static int modbus_verify_bits(void *user_ctx, int function, const uint8_t *req, int req_len,
                              int *address, int num_bits){
    int max_nb = MODBUS_MAX_READ_BITS;
    int ret = 0;
    modbus_mapping_t *mb_mapping = user_ctx;

    if( req_len < 4 ){
        return -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    *address = (req[0] << 8) + req[1];
    
    /* Note: we only need to check for DISCRETE_INPUTS, as otherwise we default to the coils
     * which are both read/write, so the same code is used to validate
     */
    int start_bits = (function == MODBUS_FC_READ_DISCRETE_INPUTS) ? mb_mapping->start_input_bits : mb_mapping->start_bits;
    int nb_bits = (function == MODBUS_FC_READ_DISCRETE_INPUTS) ? mb_mapping->nb_input_bits : mb_mapping->nb_bits;
    int mapping_address = *address - start_bits;
    if (mapping_address < 0 || (mapping_address + num_bits) > nb_bits){
        ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }

    return ret;
}

static int modbus_handle_write_coil(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *return_data, int return_data_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int address;
    int ret;

    ret = modbus_verify_bits(user_ctx, function, req, req_len, &address, 1);
    if( ret < 0 ){
        goto out;
    }

    address -= mb_mapping->start_bits;

    return_data[0] = (address & 0xFF00) >> 8;
    return_data[1] = address & 0x00FF;
    return_data[2] = 0x00;
    return_data[3] = 0x00;
    ret = 4;
    if( req[2] == 0xFF && req[3] == 0x00 ){
        mb_mapping->tab_bits[address] = 1;
        return_data[2] = 0xFF;
    }else if( req[2] == 0x00 && req[3] == 0x00 ){
        mb_mapping->tab_bits[address] = 0;
    }else{
        /* invalid value for setting coil */
        ret = -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }

out:
    return ret;
}

static int modbus_handle_write_multiple_coil(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *return_data, int return_data_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int address;
    int ret;
    int bit_count = req[2] + req[3];
    int byte_count = req[4];

    ret = modbus_verify_bits(user_ctx, function, req, req_len, &address, 1);
    if( ret < 0 ){
        goto out;
    }

    return_data[0] = (address & 0xFF00) >> 8;
    return_data[1] = address & 0x00FF;
    return_data[2] = req[2];
    return_data[3] = req[3];
    ret = 4;

    address -= mb_mapping->start_bits;

    modbus_set_bits_from_bytes(mb_mapping->tab_bits, address, bit_count, req + 5);

out:
    return ret;
}

static int modbus_handle_read_coil_di(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *rsp, int rsp_len){
    modbus_mapping_t *mb_mapping = user_ctx;
    int address;
    int ret;
    int byte_count;
    int num_bits_to_read;
    uint8_t *tab_bits = (function == MODBUS_FC_READ_DISCRETE_INPUTS) ? mb_mapping->tab_input_bits : mb_mapping->tab_bits;

    num_bits_to_read = (req[2] << 8) | req[3];
    ret = modbus_verify_bits( user_ctx, function, req, req_len, &address, num_bits_to_read );
    if( ret < 0 ){
        goto out;
    }

    if( function == MODBUS_FC_READ_DISCRETE_INPUTS ){
        address -= mb_mapping->start_input_bits;
    }else{
        address -= mb_mapping->start_bits;
    }

    byte_count = response_io_status(tab_bits, address, num_bits_to_read, rsp, 1);
    rsp[0] = byte_count;
    ret = byte_count + 1;

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
    modbus_set_function_handler(ctx, MODBUS_FC_WRITE_MULTIPLE_COILS, modbus_handle_write_multiple_coil);
    modbus_set_function_handler(ctx, MODBUS_FC_READ_DISCRETE_INPUTS, modbus_handle_read_coil_di);
    modbus_set_function_handler(ctx, MODBUS_FC_READ_HOLDING_REGISTERS, modbus_handle_read_input_holding_regs);
    modbus_set_function_handler(ctx, MODBUS_FC_READ_INPUT_REGISTERS, modbus_handle_read_input_holding_regs);
    modbus_set_function_handler(ctx, MODBUS_FC_WRITE_SINGLE_REGISTER, modbus_handle_write_register);
    modbus_set_function_handler(ctx, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, modbus_handle_write_multiple_registers);

    ret = modbus_reply_callback(ctx, req, req_length);

    modbus_set_handler_context(ctx, old_context);

    if( ret < 0 ){
        ret = -1;
    }

    if( ret > 0 ){
        ret = 0;
    }

    return ret;
}
