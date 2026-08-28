/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h> /* NOLINT(fuchsia-restrict-system-includes) */
#include <ctype.h>
#include <stdint.h> /* NOLINT(fuchsia-restrict-system-includes) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct huffman_code {
    uint8_t num_bits;
    uint32_t bits;
};

struct huffman_code_point {
    uint8_t symbol;
    struct huffman_code code;
};

enum { num_code_points = 256 };
static struct huffman_code_point code_points[num_code_points];

static size_t skip_whitespace(const char *str) {
    size_t offset = 0;
    while (str[offset] == ' ' || str[offset] == '\t') {
        ++offset;
    }
    return offset;
}

static size_t read_past_comma(const char *str) {
    size_t offset = 0;
    while (str[offset] != ',') {
        ++offset;
    }
    return offset + 1;
}

int read_code_points(const char *input_path) {

    memset(code_points, 0, sizeof(code_points));
    FILE *file = fopen(input_path, "r");
    if (!file) {
        printf("Failed to open file '%s' for read.", input_path);
        return 1;
    }

    static const char HC_KEYWORD[] = "HUFFMAN_CODE";
    static const size_t HC_KW_LEN = sizeof(HC_KEYWORD) - 1;

    int is_comment = 0;
    char line[120];
    while (fgets(line, sizeof(line), file) != NULL) {
        const size_t line_length = strlen(line);

        if (line[0] == '#') {
            /* Ignore preprocessor directives */
            continue;
        }
        for (size_t i = 0; i < line_length - 1; ++i) {
            if (!is_comment) {
                if (line[i] == '/' && line[i + 1] == '*') {
                    is_comment = 1;
                } else if (strncmp(&line[i], HC_KEYWORD, HC_KW_LEN) == 0) {
                    /* Found code, parse it */

                    /* Skip macro */
                    const char *current_char = &line[i + HC_KW_LEN];
                    current_char += skip_whitespace(current_char);
                    /* Skip ( */
                    assert(*current_char == '(');
                    ++current_char;

                    /* Parse symbol */
                    uint8_t symbol = (uint8_t)atoi(current_char);
                    struct huffman_code_point *code_point = &code_points[symbol];

                    assert(!code_point->symbol && "Symbol already found!");

                    code_point->symbol = symbol;

                    current_char += read_past_comma(current_char);
                    /* Skip the binary string form */
                    current_char += read_past_comma(current_char);

                    /* Parse bits */
                    code_point->code.bits = (uint32_t)strtol(current_char, NULL, 16);

                    current_char += read_past_comma(current_char);

                    code_point->code.num_bits = (uint8_t)atoi(current_char);
                }
            } else if (line[i] == '*' && line[i + 1] == '/') {
                is_comment = 0;
            }
        }
    }

    fclose(file);

    return 0;
}

void code_write(struct huffman_code *code, FILE *file) {

    for (int bit_idx = code->num_bits - 1; bit_idx >= 0; --bit_idx) {
        char bit = ((code->bits >> bit_idx) & 0x1) ? '1' : '0';
        fprintf(file, "%c", bit);
    }
}

struct huffman_node {
    struct huffman_code_point *value;

    struct huffman_code code;
    struct huffman_node *children[2];
};

struct huffman_node *huffman_node_new(struct huffman_code code) {

    struct huffman_node *node = malloc(sizeof(struct huffman_node));
    memset(node, 0, sizeof(struct huffman_node));
    node->code = code;
    return node;
}

struct huffman_node *huffman_node_new_value(struct huffman_code_point *value) {

    struct huffman_node *node = malloc(sizeof(struct huffman_node));
    memset(node, 0, sizeof(struct huffman_node));
    node->value = value;
    node->code = value->code;
    return node;
}

/* note: Does not actually free the memory.
   This is useful so this function may be called on the tree root. */
void huffman_node_clean_up(struct huffman_node *node) {

    for (int i = 0; i < 2; ++i) {
        if (node->children[i]) {
            huffman_node_clean_up(node->children[i]);
            free(node->children[i]);
        }
    }

    memset(node, 0, sizeof(struct huffman_node));
}

/* This function writes what to do if the pattern for node is a match */
void huffman_node_write_decode_handle_value(struct huffman_node *node, FILE *file) {

    if (!node) {
        /* Invalid node, return 0 */
        fprintf(file, "        return 0; /* invalid node */\n");
    } else if (node->value) {
        /* Attempt to inline value return */
        fprintf(
            file,
            "        *symbol = %u;\n"
            "        return %u;\n",
            node->value->symbol,
            node->value->code.num_bits);
    } else {
        /* Otherwise go to branch check */
        fprintf(file, "        goto node_");
        code_write(&node->code, file);
        fprintf(file, ";\n");
    }
}

void huffman_node_write_decode(struct huffman_node *node, FILE *file, uint8_t current_bit) {

    /* Value nodes should have been inlined into parent branch checks */
    assert(!node->value);
    assert(node->children[0] || node->children[1]);

    static int write_label = 0;

    if (write_label) {
        /* Write this node's label after the first run */
        fprintf(file, "node_");
        code_write(&node->code, file);
        fprintf(file, ":\n");
    }

    write_label = 1;

    /* Check 1 bit pattern */
    uint32_t single_bit_mask = (uint32_t)(1ull << (31 - current_bit));
    uint32_t left_aligned_pattern = ((node->code.bits << 1) + 1) << (31 - node->code.num_bits);
    uint32_t check_pattern = left_aligned_pattern & single_bit_mask;
    fprintf(file, "    if (bits & 0x%x) {\n", check_pattern);

    huffman_node_write_decode_handle_value(node->children[1], file);

    fprintf(file, "    } else {\n");

    /* Child 0 is valid, go there */
    huffman_node_write_decode_handle_value(node->children[0], file);

    fprintf(file, "    }\n\n");

    /* Recursively write child nodes */
    for (uint8_t i = 0; i < 2; ++i) {
        struct huffman_node *child = node->children[i];
        if (child && !child->value) {
            huffman_node_write_decode(child, file, current_bit + 1);
        }
    }
}

int main(int argc, char *argv[]) {

    if (argc != 4) {
        fprintf(
            stderr,
            "generator expects 3 arguments: [input file] [output file] "
            "[encoding name]\n"
            "A function of the following signature will be exported:\n"
            "struct aws_huffman_symbol_coder *[encoding name]_get_coder()\n");
        return 1;
    }

    const char *input_file = argv[1];
    const char *output_file = argv[2];
    const char *decoder_name = argv[3];

    if (read_code_points(input_file)) {
        return 1;
    }

    struct huffman_node tree_root;
    memset(&tree_root, 0, sizeof(struct huffman_node));

    /* Populate the tree */
    for (size_t i = 0; i < num_code_points; ++i) {

        struct huffman_code_point *value = &code_points[i];
        if (value->code.num_bits == 0) {
            continue;
        }

        struct huffman_node *current = &tree_root;

        uint8_t bit_idx = value->code.num_bits - 1;
        while (1) {
            struct huffman_code code = value->code;
            code.bits >>= bit_idx;
            code.num_bits = value->code.num_bits - bit_idx;

            uint8_t encoded_bit = code.bits & 0x01;
            assert(encoded_bit == 0 || encoded_bit == 1);

            if (bit_idx == 0) {
                /* Done traversing, add value as leaf */
                assert(!current->children[encoded_bit]);
                current->children[encoded_bit] = huffman_node_new_value(value);
                break;
            }

            if (current->children[encoded_bit]) {
                /* Not at the end yet, keep traversing */
                current = current->children[encoded_bit];
            } else {
                /* Not at the end yet, but this is the first time down this
                 * path. */
                struct huffman_node *new_node = huffman_node_new(code);
                current->children[encoded_bit] = new_node;
                current = new_node;
            }

            --bit_idx;
        }
    }

    /* Open the file */
    FILE *file = fopen(output_file, "w");
    if (!file) {
        printf("Failed to open file '%s' for write.", output_file);
        return 1;
    }

    /* Write the file/function header */
    fprintf(
        file,
        /* REUSE-IgnoreStart */
        "/**\n"
        " * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.\n"
        " * SPDX-License-Identifier: Apache-2.0\n"
        " */\n"
        /* REUSE-IgnoreEnd */
        "\n"
        "/* WARNING: THIS FILE WAS AUTOMATICALLY GENERATED. DO NOT EDIT. */\n"
        "/* clang-format off */\n"
        "\n"
        "#include <aws/compression/huffman.h>\n"
        "\n"
        "static struct aws_huffman_code code_points[] = {\n");

    for (size_t i = 0; i < num_code_points; ++i) {
        struct huffman_code_point *cp = &code_points[i];
        fprintf(
            file,
            "    { .pattern = 0x%x, .num_bits = %u }, /* '%c' %u */\n",
            cp->code.bits,
            cp->code.num_bits,
            isprint(cp->symbol) ? cp->symbol : ' ',
            cp->symbol);
    }

    fprintf(
        file,
        "};\n"
        "\n"
        "static struct aws_huffman_code encode_symbol(uint8_t symbol, void "
        "*userdata) {\n"
        "    (void)userdata;\n\n"
        "    return code_points[symbol];\n"
        "}\n"
        "\n"
        "/* NOLINTNEXTLINE(readability-function-size) */\n"
        "static uint8_t decode_symbol(uint32_t bits, uint8_t *symbol, void "
        "*userdata) {\n"
        "    (void)userdata;\n\n");

    /* Traverse the tree */
    huffman_node_write_decode(&tree_root, file, 0);

    /* Write the function footer & encode header */
    fprintf(
        file,
        "}\n"
        "\n"
        "struct aws_huffman_symbol_coder *%s_get_coder(void) {\n"
        "\n"
        "    static struct aws_huffman_symbol_coder coder = {\n"
        "        .encode = encode_symbol,\n"
        "        .decode = decode_symbol,\n"
        "        .userdata = NULL,\n"
        "    };\n"
        "    return &coder;\n"
        "}\n",
        decoder_name);

    fclose(file);

    huffman_node_clean_up(&tree_root);

    return 0;
}
