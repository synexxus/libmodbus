/*
 * Copyright © 2001-2013 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#ifndef MODBUS_H
#define MODBUS_H

/* Add this for macros that defined unix flavor */
#if (defined(__unix__) || defined(unix)) && !defined(USG)
#include <sys/param.h>
#endif

#ifndef _MSC_VER
#include <stdint.h>
#else
#include "stdint.h"
#endif

#include "modbus-version.h"
#include "simplelogger_defs.h"

#if defined(_MSC_VER)
# if defined(DLLBUILD)
/* define DLLBUILD when building the DLL */
#  define MODBUS_API __declspec(dllexport)
# else
#  define MODBUS_API __declspec(dllimport)
# endif
#else
# define MODBUS_API
#endif

#ifdef  __cplusplus
# define MODBUS_BEGIN_DECLS  extern "C" {
# define MODBUS_END_DECLS    }
#else
# define MODBUS_BEGIN_DECLS
# define MODBUS_END_DECLS
#endif

MODBUS_BEGIN_DECLS

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef OFF
#define OFF 0
#endif

#ifndef ON
#define ON 1
#endif

/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_SERIAL_DIAGNOSTICS        0x08
#define MODBUS_FC_GET_COMM_EVENT_COUNTER    0x0B
#define MODBUS_FC_GET_COMM_EVENT_LOG        0x0C
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_FC_REPORT_SLAVE_ID           0x11
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17
#define MODBUS_FC_READ_FIFO_QUEUE           0x18
#define MODBUS_FC_ENCAPSULATED_INTERFACE    0x2B
#define MODBUS_FC_MAX                       0x7F

/* Modbus sub-function codes */
#define MODBUS_SUBFC_DIAGNOSTIC_QUERY_DATA               0x0000
#define MDOBUS_SUBFC_DIAGNOSTIC_RESTART_COMM             0x0001
#define MODBUS_SUBFC_DIAGNOSTIC_REGISTER                 0x0002
#define MODBUS_SUBFC_DIAGNOSTIC_ASCII_DELIM              0x0003
#define MODBUS_SUBFC_DIAGNOSTIC_LISTEN_ONLY              0x0004
#define MODBUS_SUBFC_DIAGNOSTIC_CLEAR_COUNTER_DIAG       0x000A
#define MODBUS_SUBFC_DIAGNOSTIC_BUS_MSG_COUNT            0x000B
#define MODBUS_SUBFC_DIAGNOSTIC_BUS_COMM_ERR_COUNT       0x000C
#define MODBUS_SUBFC_DIAGNOSTIC_BUS_EXCEPTION_ERR_COUNT  0x000D
#define MODBUS_SUBFC_DIAGNOSTIC_SERVER_MSG_COUNT         0x000E
#define MODBUS_SUBFC_DIAGNOSTIC_SERVER_NORESPONSE_COUNT  0x000F
#define MODBUS_SUBFC_DIAGNOSTIC_SERVER_NAK_COUNT         0x0010
#define MODBUS_SUBFC_DIAGNOSTIC_SERVER_BUSY_COUNT        0x0011
#define MODBUS_SUBFC_DIAGNOSTIC_BUS_CHAR_OVERRUN_COUNT   0x0012
#define MODBUS_SUBFC_DIAGNOSTIC_CLEAR_OVERRUN_COUNT_FLAG 0x0013
#define MODBUS_SUBFC_ENCAP_TRANS_CANOPEN                 0x0D
#define MODBUS_SUBFC_ENCAP_TRANS_READ_DEVICE_IDENT       0x0E

#define MODBUS_SLAVE_ACCEPT_ALL  -2
#define MODBUS_SLAVE_INIT        -1
#define MODBUS_BROADCAST_ADDRESS  0

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 1 page 12)
 * Quantity of Coils to read (2 bytes): 1 to 2000 (0x7D0)
 * (chapter 6 section 11 page 29)
 * Quantity of Coils to write (2 bytes): 1 to 1968 (0x7B0)
 */
#define MODBUS_MAX_READ_BITS              2000
#define MODBUS_MAX_WRITE_BITS             1968

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 3 page 15)
 * Quantity of Registers to read (2 bytes): 1 to 125 (0x7D)
 * (chapter 6 section 12 page 31)
 * Quantity of Registers to write (2 bytes) 1 to 123 (0x7B)
 * (chapter 6 section 17 page 38)
 * Quantity of Registers to write in R/W registers (2 bytes) 1 to 121 (0x79)
 */
#define MODBUS_MAX_READ_REGISTERS          125
#define MODBUS_MAX_WRITE_REGISTERS         123
#define MODBUS_MAX_WR_WRITE_REGISTERS      121
#define MODBUS_MAX_WR_READ_REGISTERS       125

/* The size of the MODBUS PDU is limited by the size constraint inherited from
 * the first MODBUS implementation on Serial Line network (max. RS485 ADU = 256
 * bytes). Therefore, MODBUS PDU for serial line communication = 256 - Server
 * address (1 byte) - CRC (2 bytes) = 253 bytes.
 */
#define MODBUS_MAX_PDU_LENGTH              253

/* Consequently:
 * - RTU MODBUS ADU = 253 bytes + Server address (1 byte) + CRC (2 bytes) = 256
 *   bytes.
 * - TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes.
 * so the maximum of both backend in 260 bytes. This size can used to allocate
 * an array of bytes to store responses and it will be compatible with the two
 * backends.
 */
#define MODBUS_MAX_ADU_LENGTH              260

/* Random number to avoid errno conflicts */
#define MODBUS_ENOBASE 112345678

/* Protocol exceptions */
enum {
    MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01,
    MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE,
    MODBUS_EXCEPTION_ACKNOWLEDGE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY,
    MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE,
    MODBUS_EXCEPTION_MEMORY_PARITY,
    MODBUS_EXCEPTION_NOT_DEFINED,
    MODBUS_EXCEPTION_GATEWAY_PATH,
    MODBUS_EXCEPTION_GATEWAY_TARGET,
    MODBUS_EXCEPTION_MAX
};

#define EMBXILFUN  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_FUNCTION)
#define EMBXILADD  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS)
#define EMBXILVAL  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE)
#define EMBXSFAIL  (MODBUS_ENOBASE + MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE)
#define EMBXACK    (MODBUS_ENOBASE + MODBUS_EXCEPTION_ACKNOWLEDGE)
#define EMBXSBUSY  (MODBUS_ENOBASE + MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY)
#define EMBXNACK   (MODBUS_ENOBASE + MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE)
#define EMBXMEMPAR (MODBUS_ENOBASE + MODBUS_EXCEPTION_MEMORY_PARITY)
#define EMBXGPATH  (MODBUS_ENOBASE + MODBUS_EXCEPTION_GATEWAY_PATH)
#define EMBXGTAR   (MODBUS_ENOBASE + MODBUS_EXCEPTION_GATEWAY_TARGET)

/* Native libmodbus error codes */
#define EMBBADCRC  (EMBXGTAR + 1)
#define EMBBADDATA (EMBXGTAR + 2)
#define EMBBADEXC  (EMBXGTAR + 3)
#define EMBUNKEXC  (EMBXGTAR + 4)
#define EMBMDATA   (EMBXGTAR + 5)
#define EMBBADSLAVE (EMBXGTAR + 6)

extern const unsigned int libmodbus_version_major;
extern const unsigned int libmodbus_version_minor;
extern const unsigned int libmodbus_version_micro;

typedef struct _modbus modbus_t;

typedef struct {
    int nb_bits;
    int start_bits;
    int nb_input_bits;
    int start_input_bits;
    int nb_input_registers;
    int start_input_registers;
    int nb_registers;
    int start_registers;
    uint8_t *tab_bits;
    uint8_t *tab_input_bits;
    uint16_t *tab_input_registers;
    uint16_t *tab_registers;
} modbus_mapping_t;

typedef enum
{
    MODBUS_ERROR_RECOVERY_NONE          = 0,
    MODBUS_ERROR_RECOVERY_LINK          = (1<<1),
    MODBUS_ERROR_RECOVERY_PROTOCOL      = (1<<2)
} modbus_error_recovery_mode;

/**
 * Generic modbus handling function
 *
 * @param user_ctx The user context
 * @param slave The slave number that this request is for
 * @param function The function code that this is for
 * @param req The incoming data from the master
 * @param req_len The length of data from the master
 * @param rsp Fill this array in with the data you want to send back to the master
 * @param rsp_len How long the returned data is
 * @return length of return data if successful, (negative)MODBUS_EXCEPTION_XXXX otherwise
 */
typedef int (*modbus_handle_function)(void *user_ctx, int slave, int function, const uint8_t *req, int req_len, uint8_t *rsp, int rsp_len);

/**
 * This function should read the coils from a device.
 *
 * @param user_ctx The user's context
 * @param slave The slave number that this request is for
 * @param address The starting address for the coils
 * @param num_coils How many coils to read
 * @param bytes The return data.  This array is num_coils long; Set one bit value per byte
 * @return length of return data if successful, (negative)MODBUS_EXCEPTION_XXXX otherwise
 */ 
typedef int (*modbus_read_coil_function)(void *user_ctx, int slave, uint16_t address, int num_coils, uint8_t *bytes);
typedef int (*modbus_write_coil_function)(void *user_ctx, int slave, uint16_t address, int num_coils, uint8_t *bytes);
typedef int (*modbus_read_discrete_input_function)(void *user_ctx, int slave, uint16_t address, int num_inputs, uint8_t *bytes);
typedef int (*modbus_read_input_register_function)(void *user_ctx, int slave, uint16_t address, int num_registers, uint16_t *registers);
typedef int (*modbus_read_holding_register_function)(void *user_ctx, int slave, uint16_t address, int num_registers, uint16_t *registers);
typedef int (*modbus_write_holding_register_function)(void *user_ctx, int slave, uint16_t address, int num_registers, uint16_t *registers);

typedef struct {
    int (*verify)(void *user_ctx, int slave, int function, uint16_t address, int nb);
    int (*read)(void *user_ctx, int slave, int function, uint16_t address, int nb, uint8_t bytes[], int len);
    int (*write)(void *user_ctx, int slave, int function, uint16_t address, int nb, const uint8_t bytes[]);
} modbus_reply_callbacks_t;

MODBUS_API int modbus_set_slave(modbus_t* ctx, int slave);
MODBUS_API int modbus_get_slave(modbus_t* ctx);
MODBUS_API int modbus_set_error_recovery(modbus_t *ctx, modbus_error_recovery_mode error_recovery);
MODBUS_API int modbus_set_socket(modbus_t *ctx, int s);
MODBUS_API int modbus_get_socket(modbus_t *ctx);

MODBUS_API int modbus_get_response_timeout(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec);
MODBUS_API int modbus_set_response_timeout(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec);

MODBUS_API int modbus_get_byte_timeout(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec);
MODBUS_API int modbus_set_byte_timeout(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec);

MODBUS_API int modbus_get_indication_timeout(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec);
MODBUS_API int modbus_set_indication_timeout(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec);

MODBUS_API int modbus_get_header_length(modbus_t *ctx);

MODBUS_API int modbus_connect(modbus_t *ctx);
MODBUS_API void modbus_close(modbus_t *ctx);

MODBUS_API void modbus_free(modbus_t *ctx);

MODBUS_API int modbus_flush(modbus_t *ctx);
MODBUS_API int modbus_set_debug(modbus_t *ctx, int flag);

MODBUS_API const char *modbus_strerror(int errnum);

MODBUS_API int modbus_read_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest);
MODBUS_API int modbus_read_input_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest);
MODBUS_API int modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest);
MODBUS_API int modbus_read_input_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest);
MODBUS_API int modbus_write_bit(modbus_t *ctx, int coil_addr, int status);
MODBUS_API int modbus_write_register(modbus_t *ctx, int reg_addr, int value);
MODBUS_API int modbus_write_bits(modbus_t *ctx, int addr, int nb, const uint8_t *data);
MODBUS_API int modbus_write_registers(modbus_t *ctx, int addr, int nb, const uint16_t *data);
MODBUS_API int modbus_mask_write_register(modbus_t *ctx, int addr, uint16_t and_mask, uint16_t or_mask);
MODBUS_API int modbus_write_and_read_registers(modbus_t *ctx, int write_addr, int write_nb,
                                               const uint16_t *src, int read_addr, int read_nb,
                                               uint16_t *dest);
MODBUS_API int modbus_report_slave_id(modbus_t *ctx, int max_dest, uint8_t *dest);

MODBUS_API modbus_mapping_t* modbus_mapping_new_start_address(
    unsigned int start_bits, unsigned int nb_bits,
    unsigned int start_input_bits, unsigned int nb_input_bits,
    unsigned int start_registers, unsigned int nb_registers,
    unsigned int start_input_registers, unsigned int nb_input_registers);

MODBUS_API modbus_mapping_t* modbus_mapping_new(int nb_bits, int nb_input_bits,
                                                int nb_registers, int nb_input_registers);
MODBUS_API void modbus_mapping_free(modbus_mapping_t *mb_mapping);

MODBUS_API int modbus_send_raw_request(modbus_t *ctx, uint8_t *raw_req, int raw_req_length);

MODBUS_API int modbus_receive(modbus_t *ctx, uint8_t *req);

MODBUS_API int modbus_receive_confirmation(modbus_t *ctx, uint8_t *rsp);

MODBUS_API int modbus_reply(modbus_t *ctx, const uint8_t *req,
                            int req_length, modbus_mapping_t *mb_mapping);
MODBUS_API int modbus_reply_exception(modbus_t *ctx, const uint8_t *req,
                                      unsigned int exception_code);
MODBUS_API int modbus_set_reply_callbacks(modbus_t *ctx,
                                          const modbus_reply_callbacks_t *cb,
                                          void *user_ctx);
MODBUS_API int modbus_reply_callback(modbus_t *ctx, const uint8_t *req,
                                     int req_length);
/**
 * Callback API
 *
 * The callbacks are handled in the following order:
 * - The specific callback function is used(if available)
 * - The general callback function is used(if available)
 *
 * If there is no handler for the given function code, an ILLEGAL_FUNCTION
 * will be returned to the modbus master
 */

MODBUS_API void modbus_set_handler_context(modbus_t *ctx, void *user_ctx);
MODBUS_API void* modbus_get_handler_context(modbus_t *ctx);

/**
 * Set a custom function handler.
 * 
 * If this is NULL, deletes the handler.
 */
MODBUS_API int modbus_set_function_handler(modbus_t *ctx, int function_code,
                                           modbus_handle_function handler);

/**
 * Convert an array of uint8_t to uint16_t.
 *
 * @param req The uint8 array to convert
 * @param req_len How long the uint8 array is
 * @param out_regs The uint16 array to put data into
 * @param out_len How long the uint16 array is
 * @returns How may registers were converted, or -1 if out_regs is not large enough
 */
MODBUS_API int modbus_convert_uint8_to_uint16( const uint8_t *req, int req_len, uint16_t *out_regs, int out_len );

MODBUS_API int modbus_set_coil_handlers(modbus_t *ctx, 
                                        modbus_read_coil_function rd_handler,
                                        modbus_write_coil_function wr_handler);
MODBUS_API int modbus_set_discrete_input_handler(modbus_t *ctx,
                                        modbus_read_discrete_input_function rd_handler);
MODBUS_API int modbus_set_input_handler(modbus_t *ctx,
                                        modbus_read_input_register_function rd_handler);
MODBUS_API int modbus_set_holding_handlers(modbus_t *ctx,
                                           modbus_read_holding_register_function rd_handler,
                                           modbus_write_holding_register_function wr_handler);


/**
 * Asynchronous API
 **/

/**
 * Asynchronous method call back.  This callback is called for all register-oriented functions.
 *
 * @param ctx The context this callback is from
 * @param function_code The function code that this callback is in response to.  One of MODBUS_FC macros
 * @param addr The modbus register address the data starts out
 * @param nb The length of the dest array in terms of registers, not bytes(multiply by 2 to get total array length)
 * @param data The array of data requested from the device
 * @param error_code The error code.  If the error code is != 0, the call failed for some reason. 
 *  The data array will not contain any useful information, and nb will be 0.  This corresponds
 *  to the modbus error code(for example, MODBUS_EXCEPTION_ILLEGAL_FUNCTION)
 *  If the call times out, error_code will be -1.  Data will not gontain any useful information, and nb will be 0.
 * @param callback_data User-specified data for the callback
 */
typedef void (*modbus_async_callback_t)(modbus_t *ctx, 
                                        int function_code, 
                                        int addr, 
                                        int nb, 
                                        uint16_t *data, 
                                        int error_code,
                                        void* callback_data );

typedef void (*modbus_async_bit_callback_t)(modbus_t *ctx,
		                            int function_code,
					    int addr,
					    int nb,
					    uint8_t *data,
					    int error_code,
					    void* callback_data);

/**
 * Process modbus data, as a master device.  This must be called periodically 
 * in order to properly use the asynchronous API.
 */
MODBUS_API void modbus_process_data_master(modbus_t *ctx);

/**
 * Read registers in an async manner.  Reads the holding registers(MODBUS_FC_READ_HOLDING_REGISTERS)
 */
MODBUS_API int modbus_read_registers_async(modbus_t *ctx, int addr, int nb, 
                                           modbus_async_callback_t callback, 
                                           void* callback_data );
/**
 * Read registers in an async manner.  Reads the input registers(MODBUS_FC_READ_INPUT_REGISTERS)
 */
MODBUS_API int modbus_read_input_registers_async(modbus_t *ctx, int addr, int nb, 
                                                 modbus_async_callback_t callback, 
                                                 void* callback_data );
/**
 * Read registers in an async manner.  Reads the coil(bit) inputs(MODBUS_FC_READ_COILS)
 */
MODBUS_API int modbus_read_bits_async(modbus_t *ctx, int addr, int nb, 
                                      modbus_async_bit_callback_t callback,
                                      void* callback_data );
/**
 * Read registers in an async manner.  Reads the input(bit) inputs(MODBUS_FC_READ_INPUT_REGISTERS)
 */
MODBUS_API int modbus_read_input_bits_async(modbus_t *ctx, int addr, int nb, 
                                            modbus_async_bit_callback_t callback,
                                            void* callback_data);

/**
 * Retruns true if we are currently in an async operation, false otherwise
 */
MODBUS_API int modbus_is_in_async_operation(modbus_t *ctx);

/**
 * Process modbus data, as a slave device.  This should be called whenever there is new data 
 * that comes in on the line(serial/ethernet)
 */
MODBUS_API void modbus_process_data_slave(modbus_t *ctx);

/**
 * UTILS FUNCTIONS
 **/

#define MODBUS_GET_HIGH_BYTE(data) (((data) >> 8) & 0xFF)
#define MODBUS_GET_LOW_BYTE(data) ((data) & 0xFF)
#define MODBUS_GET_INT64_FROM_INT16(tab_int16, index) \
    (((int64_t)tab_int16[(index)    ] << 48) + \
     ((int64_t)tab_int16[(index) + 1] << 32) + \
     ((int64_t)tab_int16[(index) + 2] << 16) + \
      (int64_t)tab_int16[(index) + 3])
#define MODBUS_GET_INT32_FROM_INT16(tab_int16, index) ((tab_int16[(index)] << 16) + tab_int16[(index) + 1])
#define MODBUS_GET_INT16_FROM_INT8(tab_int8, index) ((tab_int8[(index)] << 8) + tab_int8[(index) + 1])
#define MODBUS_SET_INT16_TO_INT8(tab_int8, index, value) \
    do { \
        tab_int8[(index)] = (value) >> 8;  \
        tab_int8[(index) + 1] = (value) & 0xFF; \
    } while (0)
#define MODBUS_SET_INT32_TO_INT16(tab_int16, index, value) \
    do { \
        tab_int16[(index)    ] = (value) >> 16; \
        tab_int16[(index) + 1] = (value); \
    } while (0)
#define MODBUS_SET_INT64_TO_INT16(tab_int16, index, value) \
    do { \
        tab_int16[(index)    ] = (value) >> 48; \
        tab_int16[(index) + 1] = (value) >> 32; \
        tab_int16[(index) + 2] = (value) >> 16; \
        tab_int16[(index) + 3] = (value); \
    } while (0)

MODBUS_API void modbus_set_bits_from_byte(uint8_t *dest, int idx, const uint8_t value);
MODBUS_API void modbus_set_bits_from_bytes(uint8_t *dest, int idx, unsigned int nb_bits,
                                       const uint8_t *tab_byte);
MODBUS_API uint8_t modbus_get_byte_from_bits(const uint8_t *src, int idx, unsigned int nb_bits);
MODBUS_API void modbus_set_bytes_from_bits(const uint8_t *src, int idx, int nb_bits, uint8_t* dest);
MODBUS_API float modbus_get_float(const uint16_t *src);
MODBUS_API float modbus_get_float_abcd(const uint16_t *src);
MODBUS_API float modbus_get_float_dcba(const uint16_t *src);
MODBUS_API float modbus_get_float_badc(const uint16_t *src);
MODBUS_API float modbus_get_float_cdab(const uint16_t *src);

MODBUS_API void modbus_set_float(float f, uint16_t *dest);
MODBUS_API void modbus_set_float_abcd(float f, uint16_t *dest);
MODBUS_API void modbus_set_float_dcba(float f, uint16_t *dest);
MODBUS_API void modbus_set_float_badc(float f, uint16_t *dest);
MODBUS_API void modbus_set_float_cdab(float f, uint16_t *dest);

MODBUS_API void modbus_set_log_function( simplelogger_log_function func );

#include "modbus-tcp.h"
#include "modbus-rtu.h"

MODBUS_END_DECLS

#endif  /* MODBUS_H */
