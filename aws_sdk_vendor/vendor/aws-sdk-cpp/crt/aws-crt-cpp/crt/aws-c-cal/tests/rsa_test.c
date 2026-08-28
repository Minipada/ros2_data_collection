/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/cal/cal.h>
#include <aws/cal/hash.h>
#include <aws/cal/private/der.h>
#include <aws/cal/private/rsa.h>
#include <aws/cal/rsa.h>
#include <aws/common/encoding.h>
#include <aws/testing/aws_test_harness.h>

#include "test_case_helper.h"

/*
 * TODO: Need better test vectors. NIST ones are a pain to use.
 * For now using manually generated vectors and relying on round tripping.
 */

static int s_byte_buf_decoded_from_base64_cur(
    struct aws_allocator *allocator,
    struct aws_byte_cursor cur,
    struct aws_byte_buf *out) {
    size_t decoded_length = 0;
    ASSERT_SUCCESS(aws_base64_compute_decoded_len(&cur, &decoded_length));
    ASSERT_SUCCESS(aws_byte_buf_init(out, allocator, decoded_length));
    ASSERT_SUCCESS(aws_base64_decode(&cur, out));
    return AWS_OP_SUCCESS;
}

static const char *TEST_ENCRYPTION_STRING = "The quick brown fox jumps over the lazy dog.";

static int s_rsa_encryption_roundtrip_helper(
    struct aws_allocator *allocator,
    struct aws_rsa_key_pair *key_pair,
    enum aws_rsa_encryption_algorithm algo) {
    struct aws_byte_cursor plaintext_cur = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);

    /*short buffer should fail*/
    struct aws_byte_buf ciphertext_short;
    ASSERT_SUCCESS(aws_byte_buf_init(&ciphertext_short, allocator, 5));
    ASSERT_ERROR(AWS_ERROR_SHORT_BUFFER, aws_rsa_key_pair_encrypt(key_pair, algo, plaintext_cur, &ciphertext_short));

    /*make sure not to clobber anything in existing buffer*/
    struct aws_byte_cursor prefix = aws_byte_cursor_from_c_str("random_prefix");
    struct aws_byte_buf ciphertext;
    ASSERT_SUCCESS(aws_byte_buf_init(&ciphertext, allocator, prefix.len + aws_rsa_key_pair_block_length(key_pair)));
    ASSERT_SUCCESS(aws_byte_buf_append(&ciphertext, &prefix));
    ASSERT_SUCCESS(aws_rsa_key_pair_encrypt(key_pair, algo, plaintext_cur, &ciphertext));

    struct aws_byte_cursor ciphertext_cur = aws_byte_cursor_from_buf(&ciphertext);
    ASSERT_TRUE(aws_byte_cursor_starts_with(&ciphertext_cur, &prefix));

    aws_byte_cursor_advance(&ciphertext_cur, prefix.len);

    struct aws_byte_buf decrypted_short;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted_short, allocator, 5));
    ASSERT_ERROR(AWS_ERROR_SHORT_BUFFER, aws_rsa_key_pair_decrypt(key_pair, algo, ciphertext_cur, &decrypted_short));

    struct aws_byte_buf decrypted;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted, allocator, prefix.len + aws_rsa_key_pair_block_length(key_pair)));
    ASSERT_SUCCESS(aws_byte_buf_append(&decrypted, &prefix));
    ASSERT_SUCCESS(aws_rsa_key_pair_decrypt(key_pair, algo, ciphertext_cur, &decrypted));

    struct aws_byte_cursor decrypted_cur = aws_byte_cursor_from_buf(&decrypted);
    ASSERT_TRUE(aws_byte_cursor_starts_with(&decrypted_cur, &prefix));

    aws_byte_cursor_advance(&decrypted_cur, prefix.len);
    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(decrypted_cur, TEST_ENCRYPTION_STRING);

    aws_byte_buf_clean_up_secure(&ciphertext_short);
    aws_byte_buf_clean_up_secure(&decrypted_short);
    aws_byte_buf_clean_up_secure(&ciphertext);
    aws_byte_buf_clean_up_secure(&decrypted);

    return AWS_OP_SUCCESS;
}

static const char *TEST_PKCS1_RSA_PRIVATE_KEY_2048 = "MIIEpQIBAAKCAQEAnt/K3mvMgPQcplTgDHrItQPvXiYpUFMt5nIJbZV820Zj4n4G"
                                                     "7iB3Y9h5HzfBYga2Olr8Irv3OuKkIH0ydrdz2oBZuf7SOBQVpro3m1+oMKhcCtrI"
                                                     "GYA2MDOowaVkx6ho8pQ6K2d76pYj7GWfo0fm2p1O2jcw3JWXAPqq8dmTCMRxOw/2"
                                                     "1eB/6bto8vayljXy85WiCPm7WTZ2mhB9tvkSRijDVF+SEILdkVPPUT1eqox+me2Y"
                                                     "SM2qaXVtToscqoicOqXD8XrWFuyqeLe29CiZAA9xqmit9o/ckdNXTjiGp6cIx2qC"
                                                     "Svbkxwi7OK0BB4y1LOTVz021jbJRr9b+ZbP0zwIDAQABAoIBAQCU5+ort9uwDZyA"
                                                     "pVJtP/O3/V0v4BKez6dYsw91H0Qr/PiHg1mZfOKJuY4knUxqRSIs5bQmFgitr1jn"
                                                     "fpB6xo0WgXAXrOd5WhHE+ApAXVK1cEb8gXxEsm+XlAOapBsmKwlaO2Wd4ts4zsoS"
                                                     "ulj6X9zWj9QlIM9yH96tM0Rfc26lKoRx+jkEml24nOia6gBhnfups/Kq/sUxtnX7"
                                                     "qQTuCmIuwdDMWTnW/AYlX6+wsSRgl7iUhnoOpbl18AzaIJbgcw49yE0xp2XVRWew"
                                                     "VR86EsF7pR4hxpORgysiDmyQLLfcz85eiub3tE/A4uHUzxd37e1OwqItvrG4s/4y"
                                                     "YNiqEbz5AoGBAOSs57pjCbh92UPMeMKhPhGSdlxQ/GkjHIDpUuy5oU2ZB2akKg5Y"
                                                     "Asl51tibKsTuDR5qNtUJGEw8cMVR3A+t7p4KE7eCzRmZj/bNDBSdxnTec83Y5KcZ"
                                                     "Pqi4DktHju7mArlIhmnphqOrXvDuJoIjMGFMVNACk05loxPpg3WiCS+NAoGBALHb"
                                                     "sbP6ftkL8M9vMMRrL/Jzlz3jS0smNiJxRmW2TrpF7h0o3QeW6uzDigd1pjg40bUl"
                                                     "0/NilaWtlK4DRWQ+0FYuxrDQd3vHiZ38uoCJZVkSWxzSytFSaM0rpU2l28aWIwL0"
                                                     "ZuIk3k3l2gQBqX2VUrMFxq2MJF5ZN/OgOrUIqEDLAoGBAL7JG1TASF3qcZhFQgNw"
                                                     "L67NeX6v+sdlCeTrxcnHXjK1mB0kngn1l+2sf3mci+RdkAhuKW0391OzoYqfL3DN"
                                                     "dqXYVnbm5GOVYS1SCeAxemALMKbvbGWVhFeTqClafIAI2wDm52357eEjm0R8DRjK"
                                                     "bxTecGxTmb7wwUxdqNY96FgJAoGAHtLyYzzAiyE0pN6iVwg0kRJTXdhsjiObMjDr"
                                                     "gGkuD75a3Bbe55fSMyJYY56SJiBCx+A8cWvef44rvFS4y/zO4oDM0ovuiTc1tHm+"
                                                     "YNRvChbST5aAq/JaU2SDC4f5JNuUScjNo9e750g0lokrNKaSZJBVtHIbQ3a26bQV"
                                                     "OJa9gi0CgYEAlhk8ThPcokpqIuweY4cx34zdU1EAm/CBO1ax1Ogs/HilPFyGYXzz"
                                                     "tZI9TCH4Sq33MGEjf2MyW0XMXC56dA2VOPSTHGKaoKmyn7L9G4WfDFcYmCdvmLkR"
                                                     "7wAz2Dyxr6ImChSWD/y2ddz1U+H39uqRxwIkwJ7TbDflYNXgsAOOlUg=";

static const char *TEST_PKCS8_RSA_PRIVATE_KEY_2048 = "MIIEvwIBADANBgkqhkiG9w0BAQEFAASCBKkwggSlAgEAAoIBAQDN0jYhXzsI1lSA"
                                                     "j+3uxraUu/78uHJlCcTifQlVUftgEzJth7WNGvrJ8bDLUHwV04cTYSkydgsivjms"
                                                     "XFgzuTCOLiHX0ik/RT1fOvOpk0gte2gCfPIACUAkTlGXKJ+v+1kqnWFmZFNvtkz2"
                                                     "fmLIRq9ZrXWORVmySCeoSfvyygxkmfTUfGieFr8LMFSB6VoWKA/vOk0sMF/5PRxm"
                                                     "JESS4pOMjWQ3QHpLdjsgUAnxsThLVMbaA5JXdHnFHlgkTbL/OSY06EUzuZ0dJpTU"
                                                     "qSBbaz6qewlIO4eM9d2N+5d/mt5nn4rU7yuLGlJE/U9UpSutuvkCkqosST9yKBz7"
                                                     "YUhK7WyPAgMBAAECggEAYrHEZyhFJK2yA5wA2hjLgHLNiN3hbPXMRVbz3MfdJGrQ"
                                                     "KZmDw1AGpkORJU1I0yaFhRN4L8xO9rAE89OsL9FDqUoRzG3ofYB0N3ALW2tWlwiw"
                                                     "DVFgsge9jCtKEJPYTwjV7wtcoz7Ei7L9IM3mDGdoujXlQv2aT1UuPxKLEBc27h3Q"
                                                     "KZZL0QbA33SmO24BUx9Y741hIXsVvoce0MQM1TPwEGH18qYpffb3kCBTwFEySx+q"
                                                     "JFwadZiK1JPQgpELrXZT0VuARD2Ze8Z7c0b6668wYiwJ2mmf4l0NTLk67mIIfG1w"
                                                     "NNNzHme/UGQEjSjUU9TFWLVHU9enumPv5UeLx3wvgQKBgQD7k6pCk8kkUcjjq0sq"
                                                     "92Coy1DMlZg9ictcusJ5pfe1ZdW01iX/S/88ZOxk1X27jTWcHarAeA+ci4LzDWPG"
                                                     "I61Hy4kJFOJB6Sx3hDSnfNvuIC/7zrcWLNzvfAeDYmPhAqi1K4HYFN7Ipa/1+Uew"
                                                     "kWVllcUHRz5zWU+oYVXDsjARrwKBgQDRcJowibhaG4UUMzRsyyEK1l/w+z75L8A7"
                                                     "2ZA+wMqyUo9LPgJRYvEHHRQTHDGCzP81IUaAK2OfWrD5Jnn6r4Il7+cQ+xCHKiMD"
                                                     "rBhnL9dG4lVAWgdZPyWXfrdT3mqpZ3UgfWysvph8s48QdLA3ku7PpTfchwtSdXQP"
                                                     "cxhEItlrIQKBgQDTTB8AdCfIfXiA3+nuWH+yxbFDY5HOfeF0LNgSXDdFABcSH5si"
                                                     "Za4mB44U0ssbr2qLiM9VgIF8NiDyCxj13hk359dc7VFrknBqoXuoANKnmhkzIVfd"
                                                     "JCkca8vTqdvBrP4NzFDuL/k+BQtZSNnRjwze2X/2sPve3fBtt/LUvuBouQKBgQC9"
                                                     "T1Cv6uxN1m410grjA8C8MQXLpu5HAxh5gLBXaKBPCz0mv8gMlKhUy73ngCZomq9b"
                                                     "8NXu6ElGMw2gR10ecSHs9KohuS45XqcDnLz6GE44bkCsyDO4QdHS2+EN2A8FTNSc"
                                                     "J4Lhqe3fWdZJA5B8yz09R5P0q8RaJnxfsqMOg4mOwQKBgQC+N5xLCRa/n1kJ11we"
                                                     "rnOe2POS+zuThhi1aMn0LLPIDwVMktymV7F8JegbdYB5KjxyxzDPcpRDRKCXvAew"
                                                     "QiaCZLRyigBqDpDP4l3uIp1OEzWLYuEAWwnErC7fuPm0TFAy5ecegxW7eXasDy2C"
                                                     "dJJcK3yV8NRVOzr2voGRmr4d7w==";

static int s_rsa_encryption_roundtrip_from_user(
    struct aws_allocator *allocator,
    enum aws_rsa_encryption_algorithm algo) {
    struct aws_byte_buf key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_2048), &key_buf));

    struct aws_rsa_key_pair *key_pair =
        aws_rsa_key_pair_new_from_private_key_pkcs1(allocator, aws_byte_cursor_from_buf(&key_buf));

    ASSERT_NOT_NULL(key_pair);

    s_rsa_encryption_roundtrip_helper(allocator, key_pair, algo);

    aws_rsa_key_pair_release(key_pair);
    aws_byte_buf_clean_up_secure(&key_buf);

    return AWS_OP_SUCCESS;
}

static int s_rsa_encryption_roundtrip_from_user_pkcs8(
    struct aws_allocator *allocator,
    enum aws_rsa_encryption_algorithm algo) {
    struct aws_byte_buf key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS8_RSA_PRIVATE_KEY_2048), &key_buf));

    struct aws_rsa_key_pair *key_pair =
        aws_rsa_key_pair_new_from_private_key_pkcs8(allocator, aws_byte_cursor_from_buf(&key_buf));

    ASSERT_NOT_NULL(key_pair);

    s_rsa_encryption_roundtrip_helper(allocator, key_pair, algo);

    aws_rsa_key_pair_release(key_pair);
    aws_byte_buf_clean_up_secure(&key_buf);

    return AWS_OP_SUCCESS;
}

static int s_rsa_encryption_roundtrip_pkcs1_from_user(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    ASSERT_SUCCESS(s_rsa_encryption_roundtrip_from_user(allocator, AWS_CAL_RSA_ENCRYPTION_PKCS1_5));

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_encryption_roundtrip_pkcs1_from_user, s_rsa_encryption_roundtrip_pkcs1_from_user);

static int s_rsa_encryption_roundtrip_pkcs15_from_user_pkcs8(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    ASSERT_SUCCESS(s_rsa_encryption_roundtrip_from_user_pkcs8(allocator, AWS_CAL_RSA_ENCRYPTION_PKCS1_5));

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_encryption_roundtrip_pkcs15_from_user_pkcs8, s_rsa_encryption_roundtrip_pkcs15_from_user_pkcs8);

static int s_rsa_pkcs8_key_load_error(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_2048), &key_buf));

    struct aws_rsa_key_pair *key_pair =
        aws_rsa_key_pair_new_from_private_key_pkcs8(allocator, aws_byte_cursor_from_buf(&key_buf));

    ASSERT_NULL(key_pair);
    ASSERT_INT_EQUALS(AWS_ERROR_CAL_MALFORMED_ASN1_ENCOUNTERED, aws_last_error());

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_pkcs8_key_load_error, s_rsa_pkcs8_key_load_error);

static int s_rsa_encryption_roundtrip_oaep_sha256_from_user(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    ASSERT_SUCCESS(s_rsa_encryption_roundtrip_from_user(allocator, AWS_CAL_RSA_ENCRYPTION_OAEP_SHA256));

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_encryption_roundtrip_oaep_sha256_from_user, s_rsa_encryption_roundtrip_oaep_sha256_from_user);

static int s_rsa_encryption_roundtrip_oaep_sha512_from_user(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    ASSERT_SUCCESS(s_rsa_encryption_roundtrip_from_user(allocator, AWS_CAL_RSA_ENCRYPTION_OAEP_SHA512));

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_encryption_roundtrip_oaep_sha512_from_user, s_rsa_encryption_roundtrip_oaep_sha512_from_user);

static const char *TEST_PKCS1_RSA_PRIVATE_KEY_1024 = "MIICXAIBAAKBgQCVGG9c6uBIpv4wQMB4PJpkaUjEqa6TW5v8ebStMCnICWfpUubs"
                                                     "f6nr5nPxPsG2rw+HZSLJLAYVYfXOM9D+KyhTdpIJ7z4NXzXzem25x6H/N9WnRGjB"
                                                     "6qAOLg5Srm3uoXfulN5HVXVtncTFcJJQxgBOpT8qXLycm5k8uhm0OWP6cwIDAQAB"
                                                     "AoGAT0IRGU0G87hKUi5p4sEctho+A8XMNyuw7XNpd9OtslhFtARNHBX1p6D3q5xX"
                                                     "8Bx53dkGt/i+Nym/OOHUzPj2Uy+qprBFYK71JavQyg80h8deUsQzrKTrM45pU+U7"
                                                     "uGgEg24Mw4hQ53ky6HUJtRk/PG6osx8o4DMPU0EKSeoxqMECQQDVFC7qH3m+gHoE"
                                                     "xnz+uIR1H78NQt3sa2cnZDa0ui/Ew+UASQlDDY/xqAGYa+QAhQBMZoWJLL6AaNrj"
                                                     "FtxyKr+TAkEAsyDTfwJPFRTvqUNKKIsFHlNlzDclISHcIi00ST2bDKox7pS3aukE"
                                                     "dkytVIerIKtBMds5gjYZybAAX0cC7DHloQJAbt5VmtRN0GWhF6L/nrn7kcW27vt/"
                                                     "5WftAH4QSPEnscYL/Z4DB7Si1SaJzfk1ZV/Oy/H8QWfap43ndomKoozDqQJAX9lk"
                                                     "0kVuA53cT/oNqHwbFQsTIZ8wYvY3UKJXpAku+ivn4/3312EwXgzRgrXFwAljLUZd"
                                                     "E2vXiLCAwnrA+ZoJgQJBAI/P1XTqEAUro5aDD64JuwbvCpbAL8kkwGzf6wzrdF+f"
                                                     "0CXKkTGUEG7BGWqCr9y9nBt9KuyN1VlNbziJp+UcKVc=";

static const char *TEST_PKCS1_RSA_PUBLIC_KEY_1024 = "MIGJAoGBAJUYb1zq4Eim/jBAwHg8mmRpSMSprpNbm/x5tK0wKcgJZ+lS5ux/qevmc/"
                                                    "E+wbavD4dlIsksBhVh9c4z0P4rKFN2kgnvPg1fNfN6bbnHof831adEaMHqoA4uDlKu"
                                                    "be6hd+6U3kdVdW2dxMVwklDGAE6lPypcvJybmTy6GbQ5Y/pzAgMBAAE=";

/*
 * pkcs1 signature generator using above private key and test encryption string.
 */
static const char *TEST_RSA_SIGNATURE_PKCS1 = "Gqu9pLlPvSFIW+5ZFo9ZCxMmPR8LnAeiuYir5CfNTyraF2VPksRnCKtS6i98nwPUqzlPr"
                                              "TYJ45P3c94lQIQD3SVJ3XMSAyAEWTE2pcj0F/oPzzxLcXK9cyv2Iphe4XuBjWCOVdHgFg"
                                              "rD/yAA8b+B94AqE9U/B2+k9/C3Bz2YApo=";

static const char *TEST_RSA_SIGNATURE_PKCS1_SHA1 =
    "QJxUx+4OM+v1wh0z0PJWMeGBdvjhQHxjFCdzDLeNH6zoJpgPtFXNk6i83ff75MgPpW0m33"
    "zrrzLWfzLojZFi2KFXYu1Y39We3AfREEOyG+porTmxNQ4dJH29joXS0XNf52dJpN04Lw3WN"
    "XUHDP6eG2K71uXlsus2Tm0uBe4TF0g=";

static const char *TEST_RSA_SIGNATURE_PSS = "j//04sVoqQVSmUgH+Id0oad7OgW+hGnIqx6hjr28VnVk75Obig+n3tJGWd0r+3S4ARxf2fK"
                                            "7taVvJXISQ5aWJAYx6QRgR+25rcE96eOfi6L7ShIZIUYFzGxhc9wpUMGbqHEIhm+8QP7uNo4D"
                                            "FmaPzJMgGDKL2qhedxnjtg3p8E4=";

static const char *TEST_RSA_ENCRYPTED_PKCS1 =
    "Ng97Q53hLqC0sCNMTG6poSxXeTLVWFQJS746y1VLnDD0/IYWk/gyzhNEF0M16loaBswNLnEgL"
    "OsTVHmBaglCiEobyWBYO16HO+hrJeXK76p1GfIQ+62hSwpnxx4abqS9N2rX59ahMNSnjXZmFiQn"
    "yPDbvp2UYwUydSu6ArOM/H8=";

static const char *TEST_RSA_ENCRYPTED_OAEP256 =
    "YB9CDU8z+ViRSQRvE6z3i3mFMh1NFOgKuhcYGIhZu0wqTzVV4c6Rl+x9gMQiURkLG0q1/nAF"
    "upW5g1uo5wotJKb5GCGF8oYuMu7IemY45jBIZ3tXSz1XeZ8VHVCpBNGJBP//Pp461HI9qzaPA+mFu"
    "jBppHZTE0GLpbZeryHRgK4qPR4J+EzojiE2JrzCST8Y1xrCwvwS6QjboeorVSr8ssO8oC3HJ89klg"
    "uEq19eLTp0JP8WWnREJtGfbeIW6nGeu3KEjwnXD+A//Qk5fIxPFBV4+1kTDkLyO22ZOzCevXUAv9j"
    "97f1GRuJfS2W2KL/YXQudwX1xo5ULf1UIgpeqSQ==";

static const char *TEST_RSA_ENCRYPTED_OAEP512 =
    "Wx5SdwnG1Fc0rEIZZRibRL9iUt16NydVC4Mbok50UKWf7DnhWen4H+KZW9K6bAvXHKKZx1Sog4"
    "RAONa/rrPTWYipFgvNWEQmCHb0erEemjabx3QTu5HqJpbnU5HKAA2l7JGrV26AvyVpezJWHa3h"
    "2xWLnw5JWhqL49vaZeMwtEopr2Dz0+wsH9QZaedQmRcEwO1f2QRrVbnbYFB6wjo3VF1IY7k8Dk"
    "XiLg0m9Ivb0Gwx61gRTx0DKq3zr7CNm35E+c9ujYPdGtX0MjAJfXOHeuaspzsLVAI9gdvyZ3Ca/"
    "vdEkky9ESL7Bw4tLysuqlvc2tnVuk3LXuB3QElDC3JU+A==";

static int s_rsa_verify_signing_pkcs1_sha256(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    struct aws_byte_cursor message = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf public_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PUBLIC_KEY_1024), &public_key_buf));
    struct aws_rsa_key_pair *key_pair_public =
        aws_rsa_key_pair_new_from_public_key_pkcs1(allocator, aws_byte_cursor_from_buf(&public_key_buf));
    ASSERT_NOT_NULL(key_pair_public);

    uint8_t hash[AWS_SHA256_LEN];
    AWS_ZERO_ARRAY(hash);
    struct aws_byte_buf hash_value = aws_byte_buf_from_empty_array(hash, sizeof(hash));
    aws_sha256_compute(allocator, &message, &hash_value, 0);
    struct aws_byte_cursor hash_cur = aws_byte_cursor_from_buf(&hash_value);

    struct aws_byte_buf signature_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_RSA_SIGNATURE_PKCS1), &signature_buf));
    struct aws_byte_cursor signature_cur = aws_byte_cursor_from_buf(&signature_buf);

    ASSERT_SUCCESS(aws_rsa_key_pair_verify_signature(
        key_pair_public, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA256, hash_cur, signature_cur));

    aws_byte_buf_clean_up(&hash_value);
    aws_byte_buf_clean_up(&signature_buf);
    aws_byte_buf_clean_up(&public_key_buf);
    aws_rsa_key_pair_release(key_pair_public);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_verify_signing_pkcs1_sha256, s_rsa_verify_signing_pkcs1_sha256);

static int s_rsa_verify_signing_pkcs1_sha1(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    struct aws_byte_cursor message = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);

    aws_cal_library_init(allocator);

    struct aws_byte_buf public_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PUBLIC_KEY_1024), &public_key_buf));
    struct aws_rsa_key_pair *key_pair_public =
        aws_rsa_key_pair_new_from_public_key_pkcs1(allocator, aws_byte_cursor_from_buf(&public_key_buf));
    ASSERT_NOT_NULL(key_pair_public);

    uint8_t hash[AWS_SHA1_LEN];
    AWS_ZERO_ARRAY(hash);
    struct aws_byte_buf hash_value = aws_byte_buf_from_empty_array(hash, sizeof(hash));
    aws_sha1_compute(allocator, &message, &hash_value, 0);
    struct aws_byte_cursor hash_cur = aws_byte_cursor_from_buf(&hash_value);

    struct aws_byte_buf signature_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_RSA_SIGNATURE_PKCS1_SHA1), &signature_buf));
    struct aws_byte_cursor signature_cur = aws_byte_cursor_from_buf(&signature_buf);

    ASSERT_SUCCESS(aws_rsa_key_pair_verify_signature(
        key_pair_public, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA1, hash_cur, signature_cur));

    aws_byte_buf_clean_up(&hash_value);
    aws_byte_buf_clean_up(&signature_buf);
    aws_byte_buf_clean_up(&public_key_buf);
    aws_rsa_key_pair_release(key_pair_public);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_verify_signing_pkcs1_sha1, s_rsa_verify_signing_pkcs1_sha1);

static int s_rsa_verify_signing_pss_sha256(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    struct aws_byte_cursor message = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf public_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PUBLIC_KEY_1024), &public_key_buf));
    struct aws_rsa_key_pair *key_pair_public =
        aws_rsa_key_pair_new_from_public_key_pkcs1(allocator, aws_byte_cursor_from_buf(&public_key_buf));
    ASSERT_NOT_NULL(key_pair_public);

    uint8_t hash[AWS_SHA256_LEN];
    AWS_ZERO_ARRAY(hash);
    struct aws_byte_buf hash_value = aws_byte_buf_from_empty_array(hash, sizeof(hash));
    aws_sha256_compute(allocator, &message, &hash_value, 0);
    struct aws_byte_cursor hash_cur = aws_byte_cursor_from_buf(&hash_value);

    struct aws_byte_buf signature_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_RSA_SIGNATURE_PSS), &signature_buf));
    struct aws_byte_cursor signature_cur = aws_byte_cursor_from_buf(&signature_buf);

    ASSERT_SUCCESS(
        aws_rsa_key_pair_verify_signature(key_pair_public, AWS_CAL_RSA_SIGNATURE_PSS_SHA256, hash_cur, signature_cur));

    aws_byte_buf_clean_up(&hash_value);
    aws_byte_buf_clean_up(&signature_buf);
    aws_byte_buf_clean_up(&public_key_buf);
    aws_rsa_key_pair_release(key_pair_public);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_verify_signing_pss_sha256, s_rsa_verify_signing_pss_sha256);

static int s_rsa_decrypt_pkcs1(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf private_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_1024), &private_key_buf));
    struct aws_rsa_key_pair *key_pair_private =
        aws_rsa_key_pair_new_from_private_key_pkcs1(allocator, aws_byte_cursor_from_buf(&private_key_buf));
    ASSERT_NOT_NULL(key_pair_private);

    struct aws_byte_buf encrypted;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_RSA_ENCRYPTED_PKCS1), &encrypted));
    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted);

    struct aws_byte_buf decrypted;
    aws_byte_buf_init(&decrypted, allocator, aws_rsa_key_pair_block_length(key_pair_private));

    ASSERT_SUCCESS(
        aws_rsa_key_pair_decrypt(key_pair_private, AWS_CAL_RSA_ENCRYPTION_PKCS1_5, encrypted_cur, &decrypted));

    struct aws_byte_cursor decrypted_cur = aws_byte_cursor_from_buf(&decrypted);

    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(decrypted_cur, TEST_ENCRYPTION_STRING);

    aws_byte_buf_clean_up(&private_key_buf);
    aws_byte_buf_clean_up(&decrypted);
    aws_byte_buf_clean_up(&encrypted);
    aws_rsa_key_pair_release(key_pair_private);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_decrypt_pkcs1, s_rsa_decrypt_pkcs1);

static int s_rsa_decrypt_oaep256(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf private_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_2048), &private_key_buf));
    struct aws_rsa_key_pair *key_pair_private =
        aws_rsa_key_pair_new_from_private_key_pkcs1(allocator, aws_byte_cursor_from_buf(&private_key_buf));
    ASSERT_NOT_NULL(key_pair_private);

    struct aws_byte_buf encrypted;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_RSA_ENCRYPTED_OAEP256), &encrypted));
    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted);

    struct aws_byte_buf decrypted;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted, allocator, aws_rsa_key_pair_block_length(key_pair_private)));

    ASSERT_SUCCESS(
        aws_rsa_key_pair_decrypt(key_pair_private, AWS_CAL_RSA_ENCRYPTION_OAEP_SHA256, encrypted_cur, &decrypted));

    struct aws_byte_cursor decrypted_cur = aws_byte_cursor_from_buf(&decrypted);

    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(decrypted_cur, TEST_ENCRYPTION_STRING);

    aws_byte_buf_clean_up(&private_key_buf);
    aws_byte_buf_clean_up(&decrypted);
    aws_byte_buf_clean_up(&encrypted);
    aws_rsa_key_pair_release(key_pair_private);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_decrypt_oaep256, s_rsa_decrypt_oaep256);

static int s_rsa_decrypt_oaep512(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf private_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_2048), &private_key_buf));
    struct aws_rsa_key_pair *key_pair_private =
        aws_rsa_key_pair_new_from_private_key_pkcs1(allocator, aws_byte_cursor_from_buf(&private_key_buf));
    ASSERT_NOT_NULL(key_pair_private);

    struct aws_byte_buf encrypted;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_RSA_ENCRYPTED_OAEP512), &encrypted));
    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted);

    struct aws_byte_buf decrypted;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted, allocator, aws_rsa_key_pair_block_length(key_pair_private)));

    ASSERT_SUCCESS(
        aws_rsa_key_pair_decrypt(key_pair_private, AWS_CAL_RSA_ENCRYPTION_OAEP_SHA512, encrypted_cur, &decrypted));

    struct aws_byte_cursor decrypted_cur = aws_byte_cursor_from_buf(&decrypted);

    ASSERT_CURSOR_VALUE_CSTRING_EQUALS(decrypted_cur, TEST_ENCRYPTION_STRING);

    aws_byte_buf_clean_up(&private_key_buf);
    aws_byte_buf_clean_up(&decrypted);
    aws_byte_buf_clean_up(&encrypted);
    aws_rsa_key_pair_release(key_pair_private);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_decrypt_oaep512, s_rsa_decrypt_oaep512);

static int s_rsa_signing_roundtrip_helper(
    struct aws_allocator *allocator,
    struct aws_rsa_key_pair *key_pair_private,
    struct aws_rsa_key_pair *key_pair_public,
    enum aws_rsa_signature_algorithm algo,
    const char *expected_signature) {
    struct aws_byte_cursor message = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);

    uint8_t hash[AWS_SHA256_LEN];
    AWS_ZERO_ARRAY(hash);
    struct aws_byte_buf hash_value = aws_byte_buf_from_empty_array(hash, sizeof(hash));
    if (algo == AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA1) {
        aws_sha1_compute(allocator, &message, &hash_value, 0);
    } else {
        aws_sha256_compute(allocator, &message, &hash_value, 0);
    }

    struct aws_byte_cursor hash_cur = aws_byte_cursor_from_buf(&hash_value);

    /*since our apis work by appending to buffer, lets make sure they dont
     *clobber anything already in the buffer*/
    struct aws_byte_cursor prefix = aws_byte_cursor_from_c_str("random_prefix");
    struct aws_byte_buf signature;
    ASSERT_SUCCESS(
        aws_byte_buf_init(&signature, allocator, prefix.len + aws_rsa_key_pair_signature_length(key_pair_private)));
    ASSERT_SUCCESS(aws_byte_buf_append(&signature, &prefix));
    ASSERT_SUCCESS(aws_rsa_key_pair_sign_message(key_pair_private, algo, hash_cur, &signature));

    /*short buffer should fail*/
    struct aws_byte_buf signature_short;
    ASSERT_SUCCESS(aws_byte_buf_init(&signature_short, allocator, 5));
    ASSERT_ERROR(
        AWS_ERROR_SHORT_BUFFER, aws_rsa_key_pair_sign_message(key_pair_private, algo, hash_cur, &signature_short));

    struct aws_byte_cursor signature_cur = aws_byte_cursor_from_buf(&signature);
    ASSERT_TRUE(aws_byte_cursor_starts_with(&signature_cur, &prefix));
    aws_byte_cursor_advance(&signature_cur, prefix.len);

    if (expected_signature) {
        struct aws_byte_buf sig_b64_buf;
        size_t encoded_length = 0;
        ASSERT_SUCCESS(aws_base64_compute_encoded_len(signature.len, &encoded_length));
        ASSERT_SUCCESS(aws_byte_buf_init(&sig_b64_buf, allocator, encoded_length));
        ASSERT_SUCCESS(aws_base64_encode(&signature_cur, &sig_b64_buf));
        struct aws_byte_cursor sig_b64_cur = aws_byte_cursor_from_buf(&sig_b64_buf);
        ASSERT_CURSOR_VALUE_CSTRING_EQUALS(sig_b64_cur, expected_signature);
        aws_byte_buf_clean_up_secure(&sig_b64_buf);
    }

    ASSERT_SUCCESS(aws_rsa_key_pair_verify_signature(key_pair_public, algo, hash_cur, signature_cur));

    aws_byte_buf_clean_up_secure(&signature);
    aws_byte_buf_clean_up_secure(&signature_short);

    return AWS_OP_SUCCESS;
}

static int s_rsa_signing_roundtrip_from_user(
    struct aws_allocator *allocator,
    enum aws_rsa_signature_algorithm algo,
    const char *expected_signature) {

    struct aws_byte_buf private_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_1024), &private_key_buf));
    struct aws_rsa_key_pair *key_pair_private =
        aws_rsa_key_pair_new_from_private_key_pkcs1(allocator, aws_byte_cursor_from_buf(&private_key_buf));
    ASSERT_NOT_NULL(key_pair_private);

    struct aws_byte_buf public_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PUBLIC_KEY_1024), &public_key_buf));
    struct aws_rsa_key_pair *key_pair_public =
        aws_rsa_key_pair_new_from_public_key_pkcs1(allocator, aws_byte_cursor_from_buf(&public_key_buf));
    ASSERT_NOT_NULL(key_pair_public);

    s_rsa_signing_roundtrip_helper(allocator, key_pair_private, key_pair_public, algo, expected_signature);

    aws_rsa_key_pair_release(key_pair_private);
    aws_rsa_key_pair_release(key_pair_public);
    aws_byte_buf_clean_up_secure(&private_key_buf);
    aws_byte_buf_clean_up_secure(&public_key_buf);

    return AWS_OP_SUCCESS;
}

static int s_rsa_signing_roundtrip_pkcs1_sha256_from_user(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    ASSERT_SUCCESS(
        s_rsa_signing_roundtrip_from_user(allocator, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA256, TEST_RSA_SIGNATURE_PKCS1));

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_signing_roundtrip_pkcs1_sha256_from_user, s_rsa_signing_roundtrip_pkcs1_sha256_from_user);

static int s_rsa_signing_roundtrip_pkcs1_sha1_from_user(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_init(allocator);

    ASSERT_SUCCESS(s_rsa_signing_roundtrip_from_user(
        allocator, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA1, TEST_RSA_SIGNATURE_PKCS1_SHA1));

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_signing_roundtrip_pkcs1_sha1_from_user, s_rsa_signing_roundtrip_pkcs1_sha1_from_user);

static int s_rsa_signing_roundtrip_pss_sha256_from_user(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

#if defined(AWS_OS_MACOS)
    if (__builtin_available(macOS 10.12, *)) {
        ASSERT_SUCCESS(s_rsa_signing_roundtrip_from_user(allocator, AWS_CAL_RSA_SIGNATURE_PSS_SHA256, NULL));
    } else {
        ASSERT_ERROR(
            AWS_ERROR_CAL_UNSUPPORTED_ALGORITHM,
            s_rsa_signing_roundtrip_from_user(allocator, AWS_CAL_RSA_SIGNATURE_PSS_SHA256, NULL));
    }
#else
    ASSERT_SUCCESS(s_rsa_signing_roundtrip_from_user(allocator, AWS_CAL_RSA_SIGNATURE_PSS_SHA256, NULL));
#endif

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_signing_roundtrip_pss_sha256_from_user, s_rsa_signing_roundtrip_pss_sha256_from_user);

static int s_rsa_getters(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf private_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_1024), &private_key_buf));
    struct aws_rsa_key_pair *key_pair_private =
        aws_rsa_key_pair_new_from_private_key_pkcs1(allocator, aws_byte_cursor_from_buf(&private_key_buf));

    ASSERT_NOT_NULL(key_pair_private);
    ASSERT_INT_EQUALS(128, aws_rsa_key_pair_block_length(key_pair_private));
    ASSERT_INT_EQUALS(128, aws_rsa_key_pair_signature_length(key_pair_private));

    struct aws_byte_buf priv_key;
    ASSERT_SUCCESS(aws_rsa_key_pair_get_private_key(key_pair_private, AWS_CAL_RSA_KEY_EXPORT_PKCS1, &priv_key));
    ASSERT_TRUE(priv_key.len > 0);

    struct aws_byte_buf public_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PUBLIC_KEY_1024), &public_key_buf));
    struct aws_rsa_key_pair *key_pair_public =
        aws_rsa_key_pair_new_from_public_key_pkcs1(allocator, aws_byte_cursor_from_buf(&public_key_buf));

    ASSERT_INT_EQUALS(128, aws_rsa_key_pair_block_length(key_pair_public));
    ASSERT_INT_EQUALS(128, aws_rsa_key_pair_signature_length(key_pair_public));

    struct aws_byte_buf pub_key;
    ASSERT_SUCCESS(aws_rsa_key_pair_get_public_key(key_pair_public, AWS_CAL_RSA_KEY_EXPORT_PKCS1, &pub_key));
    ASSERT_TRUE(pub_key.len > 0);

    aws_rsa_key_pair_release(key_pair_private);
    aws_rsa_key_pair_release(key_pair_public);
    aws_byte_buf_clean_up_secure(&private_key_buf);
    aws_byte_buf_clean_up_secure(&public_key_buf);
    aws_byte_buf_clean_up_secure(&priv_key);
    aws_byte_buf_clean_up_secure(&pub_key);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_getters, s_rsa_getters);

static int s_rsa_private_pkcs1_der_parsing(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    static uint8_t n[] = {0x95, 0x18, 0x6f, 0x5c, 0xea, 0xe0, 0x48, 0xa6, 0xfe, 0x30, 0x40, 0xc0, 0x78, 0x3c, 0x9a,
                          0x64, 0x69, 0x48, 0xc4, 0xa9, 0xae, 0x93, 0x5b, 0x9b, 0xfc, 0x79, 0xb4, 0xad, 0x30, 0x29,
                          0xc8, 0x09, 0x67, 0xe9, 0x52, 0xe6, 0xec, 0x7f, 0xa9, 0xeb, 0xe6, 0x73, 0xf1, 0x3e, 0xc1,
                          0xb6, 0xaf, 0x0f, 0x87, 0x65, 0x22, 0xc9, 0x2c, 0x06, 0x15, 0x61, 0xf5, 0xce, 0x33, 0xd0,
                          0xfe, 0x2b, 0x28, 0x53, 0x76, 0x92, 0x09, 0xef, 0x3e, 0x0d, 0x5f, 0x35, 0xf3, 0x7a, 0x6d,
                          0xb9, 0xc7, 0xa1, 0xff, 0x37, 0xd5, 0xa7, 0x44, 0x68, 0xc1, 0xea, 0xa0, 0x0e, 0x2e, 0x0e,
                          0x52, 0xae, 0x6d, 0xee, 0xa1, 0x77, 0xee, 0x94, 0xde, 0x47, 0x55, 0x75, 0x6d, 0x9d, 0xc4,
                          0xc5, 0x70, 0x92, 0x50, 0xc6, 0x00, 0x4e, 0xa5, 0x3f, 0x2a, 0x5c, 0xbc, 0x9c, 0x9b, 0x99,
                          0x3c, 0xba, 0x19, 0xb4, 0x39, 0x63, 0xfa, 0x73};

    static uint8_t e[] = {0x01, 0x00, 0x01};

    static uint8_t d[] = {0x4f, 0x42, 0x11, 0x19, 0x4d, 0x06, 0xf3, 0xb8, 0x4a, 0x52, 0x2e, 0x69, 0xe2, 0xc1, 0x1c,
                          0xb6, 0x1a, 0x3e, 0x03, 0xc5, 0xcc, 0x37, 0x2b, 0xb0, 0xed, 0x73, 0x69, 0x77, 0xd3, 0xad,
                          0xb2, 0x58, 0x45, 0xb4, 0x04, 0x4d, 0x1c, 0x15, 0xf5, 0xa7, 0xa0, 0xf7, 0xab, 0x9c, 0x57,
                          0xf0, 0x1c, 0x79, 0xdd, 0xd9, 0x06, 0xb7, 0xf8, 0xbe, 0x37, 0x29, 0xbf, 0x38, 0xe1, 0xd4,
                          0xcc, 0xf8, 0xf6, 0x53, 0x2f, 0xaa, 0xa6, 0xb0, 0x45, 0x60, 0xae, 0xf5, 0x25, 0xab, 0xd0,
                          0xca, 0x0f, 0x34, 0x87, 0xc7, 0x5e, 0x52, 0xc4, 0x33, 0xac, 0xa4, 0xeb, 0x33, 0x8e, 0x69,
                          0x53, 0xe5, 0x3b, 0xb8, 0x68, 0x04, 0x83, 0x6e, 0x0c, 0xc3, 0x88, 0x50, 0xe7, 0x79, 0x32,
                          0xe8, 0x75, 0x09, 0xb5, 0x19, 0x3f, 0x3c, 0x6e, 0xa8, 0xb3, 0x1f, 0x28, 0xe0, 0x33, 0x0f,
                          0x53, 0x41, 0x0a, 0x49, 0xea, 0x31, 0xa8, 0xc1};

    static uint8_t p[] = {0xd5, 0x14, 0x2e, 0xea, 0x1f, 0x79, 0xbe, 0x80, 0x7a, 0x04, 0xc6, 0x7c, 0xfe,
                          0xb8, 0x84, 0x75, 0x1f, 0xbf, 0x0d, 0x42, 0xdd, 0xec, 0x6b, 0x67, 0x27, 0x64,
                          0x36, 0xb4, 0xba, 0x2f, 0xc4, 0xc3, 0xe5, 0x00, 0x49, 0x09, 0x43, 0x0d, 0x8f,
                          0xf1, 0xa8, 0x01, 0x98, 0x6b, 0xe4, 0x00, 0x85, 0x00, 0x4c, 0x66, 0x85, 0x89,
                          0x2c, 0xbe, 0x80, 0x68, 0xda, 0xe3, 0x16, 0xdc, 0x72, 0x2a, 0xbf, 0x93};

    static uint8_t q[] = {0xb3, 0x20, 0xd3, 0x7f, 0x02, 0x4f, 0x15, 0x14, 0xef, 0xa9, 0x43, 0x4a, 0x28,
                          0x8b, 0x05, 0x1e, 0x53, 0x65, 0xcc, 0x37, 0x25, 0x21, 0x21, 0xdc, 0x22, 0x2d,
                          0x34, 0x49, 0x3d, 0x9b, 0x0c, 0xaa, 0x31, 0xee, 0x94, 0xb7, 0x6a, 0xe9, 0x04,
                          0x76, 0x4c, 0xad, 0x54, 0x87, 0xab, 0x20, 0xab, 0x41, 0x31, 0xdb, 0x39, 0x82,
                          0x36, 0x19, 0xc9, 0xb0, 0x00, 0x5f, 0x47, 0x02, 0xec, 0x31, 0xe5, 0xa1};

    static uint8_t dmp1[] = {0x6e, 0xde, 0x55, 0x9a, 0xd4, 0x4d, 0xd0, 0x65, 0xa1, 0x17, 0xa2, 0xff, 0x9e,
                             0xb9, 0xfb, 0x91, 0xc5, 0xb6, 0xee, 0xfb, 0x7f, 0xe5, 0x67, 0xed, 0x00, 0x7e,
                             0x10, 0x48, 0xf1, 0x27, 0xb1, 0xc6, 0x0b, 0xfd, 0x9e, 0x03, 0x07, 0xb4, 0xa2,
                             0xd5, 0x26, 0x89, 0xcd, 0xf9, 0x35, 0x65, 0x5f, 0xce, 0xcb, 0xf1, 0xfc, 0x41,
                             0x67, 0xda, 0xa7, 0x8d, 0xe7, 0x76, 0x89, 0x8a, 0xa2, 0x8c, 0xc3, 0xa9};

    static uint8_t dmq1[] = {0x5f, 0xd9, 0x64, 0xd2, 0x45, 0x6e, 0x03, 0x9d, 0xdc, 0x4f, 0xfa, 0x0d, 0xa8,
                             0x7c, 0x1b, 0x15, 0x0b, 0x13, 0x21, 0x9f, 0x30, 0x62, 0xf6, 0x37, 0x50, 0xa2,
                             0x57, 0xa4, 0x09, 0x2e, 0xfa, 0x2b, 0xe7, 0xe3, 0xfd, 0xf7, 0xd7, 0x61, 0x30,
                             0x5e, 0x0c, 0xd1, 0x82, 0xb5, 0xc5, 0xc0, 0x09, 0x63, 0x2d, 0x46, 0x5d, 0x13,
                             0x6b, 0xd7, 0x88, 0xb0, 0x80, 0xc2, 0x7a, 0xc0, 0xf9, 0x9a, 0x09, 0x81};

    static uint8_t iqmp[] = {0x8f, 0xcf, 0xd5, 0x74, 0xea, 0x10, 0x05, 0x2b, 0xa3, 0x96, 0x83, 0x0f, 0xae,
                             0x09, 0xbb, 0x06, 0xef, 0x0a, 0x96, 0xc0, 0x2f, 0xc9, 0x24, 0xc0, 0x6c, 0xdf,
                             0xeb, 0x0c, 0xeb, 0x74, 0x5f, 0x9f, 0xd0, 0x25, 0xca, 0x91, 0x31, 0x94, 0x10,
                             0x6e, 0xc1, 0x19, 0x6a, 0x82, 0xaf, 0xdc, 0xbd, 0x9c, 0x1b, 0x7d, 0x2a, 0xec,
                             0x8d, 0xd5, 0x59, 0x4d, 0x6f, 0x38, 0x89, 0xa7, 0xe5, 0x1c, 0x29, 0x57};

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf private_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_1024), &private_key_buf));

    struct aws_byte_cursor private_key_cur = aws_byte_cursor_from_buf(&private_key_buf);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, private_key_cur);
    struct aws_rsa_private_key_pkcs1 private_key_data;
    AWS_ZERO_STRUCT(private_key_data);
    ASSERT_SUCCESS(aws_der_decoder_load_private_rsa_pkcs1(decoder, &private_key_data));

    ASSERT_BIN_ARRAYS_EQUALS(n, AWS_ARRAY_SIZE(n), private_key_data.modulus.ptr, private_key_data.modulus.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        e, AWS_ARRAY_SIZE(e), private_key_data.publicExponent.ptr, private_key_data.publicExponent.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        d, AWS_ARRAY_SIZE(d), private_key_data.privateExponent.ptr, private_key_data.privateExponent.len);
    ASSERT_BIN_ARRAYS_EQUALS(p, AWS_ARRAY_SIZE(p), private_key_data.prime1.ptr, private_key_data.prime1.len);
    ASSERT_BIN_ARRAYS_EQUALS(q, AWS_ARRAY_SIZE(q), private_key_data.prime2.ptr, private_key_data.prime2.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        dmp1, AWS_ARRAY_SIZE(dmp1), private_key_data.exponent1.ptr, private_key_data.exponent1.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        dmq1, AWS_ARRAY_SIZE(dmq1), private_key_data.exponent2.ptr, private_key_data.exponent2.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        iqmp, AWS_ARRAY_SIZE(iqmp), private_key_data.coefficient.ptr, private_key_data.coefficient.len);

    aws_byte_buf_clean_up_secure(&private_key_buf);
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_private_pkcs1_der_parsing, s_rsa_private_pkcs1_der_parsing);

static int s_rsa_public_pkcs1_der_parsing(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    static uint8_t n[] = {0x95, 0x18, 0x6f, 0x5c, 0xea, 0xe0, 0x48, 0xa6, 0xfe, 0x30, 0x40, 0xc0, 0x78, 0x3c, 0x9a,
                          0x64, 0x69, 0x48, 0xc4, 0xa9, 0xae, 0x93, 0x5b, 0x9b, 0xfc, 0x79, 0xb4, 0xad, 0x30, 0x29,
                          0xc8, 0x09, 0x67, 0xe9, 0x52, 0xe6, 0xec, 0x7f, 0xa9, 0xeb, 0xe6, 0x73, 0xf1, 0x3e, 0xc1,
                          0xb6, 0xaf, 0x0f, 0x87, 0x65, 0x22, 0xc9, 0x2c, 0x06, 0x15, 0x61, 0xf5, 0xce, 0x33, 0xd0,
                          0xfe, 0x2b, 0x28, 0x53, 0x76, 0x92, 0x09, 0xef, 0x3e, 0x0d, 0x5f, 0x35, 0xf3, 0x7a, 0x6d,
                          0xb9, 0xc7, 0xa1, 0xff, 0x37, 0xd5, 0xa7, 0x44, 0x68, 0xc1, 0xea, 0xa0, 0x0e, 0x2e, 0x0e,
                          0x52, 0xae, 0x6d, 0xee, 0xa1, 0x77, 0xee, 0x94, 0xde, 0x47, 0x55, 0x75, 0x6d, 0x9d, 0xc4,
                          0xc5, 0x70, 0x92, 0x50, 0xc6, 0x00, 0x4e, 0xa5, 0x3f, 0x2a, 0x5c, 0xbc, 0x9c, 0x9b, 0x99,
                          0x3c, 0xba, 0x19, 0xb4, 0x39, 0x63, 0xfa, 0x73};

    static uint8_t e[] = {0x01, 0x00, 0x01};

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf public_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PUBLIC_KEY_1024), &public_key_buf));

    struct aws_byte_cursor public_key_cur = aws_byte_cursor_from_buf(&public_key_buf);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, public_key_cur);
    struct aws_rsa_public_key_pkcs1 public_key_data;
    AWS_ZERO_STRUCT(public_key_data);
    ASSERT_SUCCESS(aws_der_decoder_load_public_rsa_pkcs1(decoder, &public_key_data));

    ASSERT_BIN_ARRAYS_EQUALS(n, AWS_ARRAY_SIZE(n), public_key_data.modulus.ptr, public_key_data.modulus.len);
    ASSERT_BIN_ARRAYS_EQUALS(
        e, AWS_ARRAY_SIZE(e), public_key_data.publicExponent.ptr, public_key_data.publicExponent.len);

    aws_byte_buf_clean_up_secure(&public_key_buf);
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_public_pkcs1_der_parsing, s_rsa_public_pkcs1_der_parsing);

static int s_rsa_signing_mismatch_pkcs1_sha256(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    struct aws_byte_cursor message = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);

    aws_cal_library_test_init(allocator);

    struct aws_byte_buf public_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_1024), &public_key_buf));
    struct aws_rsa_key_pair *key_pair_private =
        aws_rsa_key_pair_new_from_private_key_pkcs1(allocator, aws_byte_cursor_from_buf(&public_key_buf));
    ASSERT_NOT_NULL(key_pair_private);

    uint8_t hash[AWS_SHA256_LEN];
    AWS_ZERO_ARRAY(hash);
    struct aws_byte_buf hash_value = aws_byte_buf_from_empty_array(hash, sizeof(hash));
    aws_sha256_compute(allocator, &message, &hash_value, 0);
    struct aws_byte_cursor hash_cur = aws_byte_cursor_from_buf(&hash_value);

    struct aws_byte_buf signature_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&signature_buf, allocator, aws_rsa_key_pair_signature_length(key_pair_private)));
    ASSERT_SUCCESS(aws_rsa_key_pair_sign_message(
        key_pair_private, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA256, hash_cur, &signature_buf));
    struct aws_byte_cursor signature_cur = aws_byte_cursor_from_buf(&signature_buf);

    hash[5] += 59; /* modify digest to force signature mismatch */

    ASSERT_ERROR(
        AWS_ERROR_CAL_SIGNATURE_VALIDATION_FAILED,
        aws_rsa_key_pair_verify_signature(
            key_pair_private, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA256, hash_cur, signature_cur));

    hash[5] -= 59; /* undo digest modification and corrupt signature */
    signature_buf.buffer[5] += 59;
    ASSERT_ERROR(
        AWS_ERROR_CAL_SIGNATURE_VALIDATION_FAILED,
        aws_rsa_key_pair_verify_signature(
            key_pair_private, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA256, hash_cur, signature_cur));

    struct aws_byte_cursor short_signature_cur = aws_byte_cursor_from_c_str("bad signature");
    ASSERT_ERROR(
        AWS_ERROR_CAL_SIGNATURE_VALIDATION_FAILED,
        aws_rsa_key_pair_verify_signature(
            key_pair_private, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA256, hash_cur, short_signature_cur));

    aws_byte_buf_clean_up(&hash_value);
    aws_byte_buf_clean_up(&signature_buf);
    aws_byte_buf_clean_up(&public_key_buf);
    aws_rsa_key_pair_release(key_pair_private);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_signing_mismatch_pkcs1_sha256, s_rsa_signing_mismatch_pkcs1_sha256);

static int s_rsa_signing_mismatch_pkcs1_sha1(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    struct aws_byte_cursor message = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);

    aws_cal_library_init(allocator);

    struct aws_byte_buf public_key_buf;
    ASSERT_SUCCESS(s_byte_buf_decoded_from_base64_cur(
        allocator, aws_byte_cursor_from_c_str(TEST_PKCS1_RSA_PRIVATE_KEY_1024), &public_key_buf));
    struct aws_rsa_key_pair *key_pair_private =
        aws_rsa_key_pair_new_from_private_key_pkcs1(allocator, aws_byte_cursor_from_buf(&public_key_buf));
    ASSERT_NOT_NULL(key_pair_private);

    uint8_t hash[AWS_SHA1_LEN];
    AWS_ZERO_ARRAY(hash);
    struct aws_byte_buf hash_value = aws_byte_buf_from_empty_array(hash, sizeof(hash));
    aws_sha1_compute(allocator, &message, &hash_value, 0);
    struct aws_byte_cursor hash_cur = aws_byte_cursor_from_buf(&hash_value);

    struct aws_byte_buf signature_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&signature_buf, allocator, aws_rsa_key_pair_signature_length(key_pair_private)));
    ASSERT_SUCCESS(
        aws_rsa_key_pair_sign_message(key_pair_private, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA1, hash_cur, &signature_buf));
    struct aws_byte_cursor signature_cur = aws_byte_cursor_from_buf(&signature_buf);

    hash[5] += 59; /* modify digest to force signature mismatch */

    ASSERT_ERROR(
        AWS_ERROR_CAL_SIGNATURE_VALIDATION_FAILED,
        aws_rsa_key_pair_verify_signature(
            key_pair_private, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA1, hash_cur, signature_cur));

    hash[5] -= 59; /* undo digest modification and corrupt signature */
    signature_buf.buffer[5] += 59;
    ASSERT_ERROR(
        AWS_ERROR_CAL_SIGNATURE_VALIDATION_FAILED,
        aws_rsa_key_pair_verify_signature(
            key_pair_private, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA1, hash_cur, signature_cur));

    struct aws_byte_cursor short_signature_cur = aws_byte_cursor_from_c_str("bad signature");
    ASSERT_ERROR(
        AWS_ERROR_CAL_SIGNATURE_VALIDATION_FAILED,
        aws_rsa_key_pair_verify_signature(
            key_pair_private, AWS_CAL_RSA_SIGNATURE_PKCS1_5_SHA1, hash_cur, short_signature_cur));

    aws_byte_buf_clean_up(&hash_value);
    aws_byte_buf_clean_up(&signature_buf);
    aws_byte_buf_clean_up(&public_key_buf);
    aws_rsa_key_pair_release(key_pair_private);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(rsa_signing_mismatch_pkcs1_sha1, s_rsa_signing_mismatch_pkcs1_sha1);
