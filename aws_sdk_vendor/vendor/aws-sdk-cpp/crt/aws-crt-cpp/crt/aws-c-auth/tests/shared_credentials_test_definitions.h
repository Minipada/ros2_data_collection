#ifndef SHARED_CREDENTIALS_TEST_DEFINITIONS_H
#define SHARED_CREDENTIALS_TEST_DEFINITIONS_H
/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/common/string.h>
#include <aws/common/uuid.h>
#include <aws/io/file_utils.h>

#include <errno.h>

#ifdef _MSC_VER
/* fopen, fprintf etc... */
#    pragma warning(push)
#    pragma warning(disable : 4996)
#endif

AWS_STATIC_STRING_FROM_LITERAL(s_default_profile_env_variable_name, "AWS_PROFILE");
AWS_STATIC_STRING_FROM_LITERAL(s_default_config_path_env_variable_name, "AWS_CONFIG_FILE");
AWS_STATIC_STRING_FROM_LITERAL(s_default_credentials_path_env_variable_name, "AWS_SHARED_CREDENTIALS_FILE");
AWS_STATIC_STRING_FROM_LITERAL(s_access_key_id_env_var, "AWS_ACCESS_KEY_ID");
AWS_STATIC_STRING_FROM_LITERAL(s_secret_access_key_env_var, "AWS_SECRET_ACCESS_KEY");
AWS_STATIC_STRING_FROM_LITERAL(s_session_token_env_var, "AWS_SESSION_TOKEN");

static struct aws_string *aws_create_process_unique_file_name(struct aws_allocator *allocator) {
    char file_name_storage[64] = {0};
    struct aws_byte_buf filename_buf = aws_byte_buf_from_empty_array(file_name_storage, sizeof(file_name_storage));

#ifndef WIN32
    AWS_FATAL_ASSERT(aws_byte_buf_write_from_whole_cursor(&filename_buf, aws_byte_cursor_from_c_str("./")));
#endif

    AWS_FATAL_ASSERT(
        aws_byte_buf_write_from_whole_cursor(&filename_buf, aws_byte_cursor_from_c_str("config_creds_test")));

    struct aws_uuid uuid;
    AWS_FATAL_ASSERT(aws_uuid_init(&uuid) == AWS_OP_SUCCESS);
    AWS_FATAL_ASSERT(aws_uuid_to_str(&uuid, &filename_buf) == AWS_OP_SUCCESS);

    return aws_string_new_from_array(allocator, filename_buf.buffer, filename_buf.len);
}

static int aws_create_profile_file(const struct aws_string *file_name, const struct aws_string *file_contents) {
    /* avoid compiler warning if some files include this header but don't actually use those variables */
    (void)s_default_profile_env_variable_name;
    (void)s_default_config_path_env_variable_name;
    (void)s_default_credentials_path_env_variable_name;
    (void)s_access_key_id_env_var;
    (void)s_secret_access_key_env_var;
    (void)s_session_token_env_var;

    FILE *fp = fopen(aws_string_c_str(file_name), "w");
    if (fp == NULL) {
        return aws_translate_and_raise_io_error(errno);
    }

    int result = fprintf(fp, "%s", aws_string_c_str(file_contents));
    fclose(fp);

    if (result < 0) {
        return aws_translate_and_raise_io_error(errno);
    }

    return AWS_OP_SUCCESS;
}

#ifdef _MSC_VER
#    pragma warning(pop)
#endif

#endif /* SHARED_CREDENTIALS_TEST_DEFINITIONS_H */
