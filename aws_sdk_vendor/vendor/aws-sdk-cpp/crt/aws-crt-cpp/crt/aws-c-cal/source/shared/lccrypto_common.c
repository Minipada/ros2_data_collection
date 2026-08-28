/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/cal/cal.h>
#include <aws/cal/private/opensslcrypto_common.h>

#define OPENSSL_SUPPRESS_DEPRECATED
#include <openssl/err.h>
#include <openssl/evp.h>

#if defined(OPENSSL_IS_OPENSSL)
/*Error defines were part of evp.h in 1.0.x and were moved to evperr.h in 1.1.0*/
#    if OPENSSL_VERSION_NUMBER >= 0x10100000L
#        include <openssl/evperr.h>
#    endif
#else
#    include <openssl/evp_errors.h>
#endif

/*
 * Transforms evp error code into crt error code and raises it as necessary.
 * All evp functions follow the same:
 * >= 1 for success
 * <= 0 for failure
 * -2 always indicates incorrect algo for operation
 */
int aws_reinterpret_lc_evp_error_as_crt(int evp_error, const char *function_name, enum aws_cal_log_subject subject) {
    if (evp_error > 0) {
        return AWS_OP_SUCCESS;
    }

    /* AWS-LC/BoringSSL error code is uint32_t, but OpenSSL uses unsigned long. */
#if defined(OPENSSL_IS_OPENSSL)
    uint32_t error = ERR_peek_error();
#else
    unsigned long error = ERR_peek_error();
#endif

    int crt_error = AWS_OP_ERR;
    const char *error_message = ERR_reason_error_string(error);

    if (evp_error == -2) {
        crt_error = AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM;
        goto on_error;
    }

    if (ERR_GET_LIB(error) == ERR_LIB_EVP) {
        switch (ERR_GET_REASON(error)) {
            case EVP_R_BUFFER_TOO_SMALL: {
                crt_error = AWS_ERROR_SHORT_BUFFER;
                goto on_error;
            }
            case EVP_R_UNSUPPORTED_ALGORITHM: {
                crt_error = AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM;
                goto on_error;
            }
        }
    }

    crt_error = AWS_ERROR_CAL_CRYPTO_OPERATION_FAILED;

on_error:
    AWS_LOGF_ERROR(
        subject,
        "%s() failed. returned: %d extended error:%lu(%s) aws_error:%s",
        function_name,
        evp_error,
        (unsigned long)error,
        error_message == NULL ? "" : error_message,
        aws_error_name(crt_error));

    return aws_raise_error(crt_error);
}
