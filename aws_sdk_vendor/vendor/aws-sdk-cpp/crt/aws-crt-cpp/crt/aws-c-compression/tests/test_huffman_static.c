/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

/* WARNING: THIS FILE WAS AUTOMATICALLY GENERATED. DO NOT EDIT. */

#include <aws/compression/huffman.h>

static struct aws_huffman_code code_points[] = {
    {.pattern = 0x32e, .num_bits = 10}, /* ' ' 0 */
    {.pattern = 0x32f, .num_bits = 10}, /* ' ' 1 */
    {.pattern = 0x330, .num_bits = 10}, /* ' ' 2 */
    {.pattern = 0x331, .num_bits = 10}, /* ' ' 3 */
    {.pattern = 0x332, .num_bits = 10}, /* ' ' 4 */
    {.pattern = 0x333, .num_bits = 10}, /* ' ' 5 */
    {.pattern = 0x334, .num_bits = 10}, /* ' ' 6 */
    {.pattern = 0x335, .num_bits = 10}, /* ' ' 7 */
    {.pattern = 0x336, .num_bits = 10}, /* ' ' 8 */
    {.pattern = 0x337, .num_bits = 10}, /* ' ' 9 */
    {.pattern = 0xb8, .num_bits = 8},   /* ' ' 10 */
    {.pattern = 0x338, .num_bits = 10}, /* ' ' 11 */
    {.pattern = 0x339, .num_bits = 10}, /* ' ' 12 */
    {.pattern = 0x33a, .num_bits = 10}, /* ' ' 13 */
    {.pattern = 0x33b, .num_bits = 10}, /* ' ' 14 */
    {.pattern = 0x33c, .num_bits = 10}, /* ' ' 15 */
    {.pattern = 0x33d, .num_bits = 10}, /* ' ' 16 */
    {.pattern = 0x33e, .num_bits = 10}, /* ' ' 17 */
    {.pattern = 0x33f, .num_bits = 10}, /* ' ' 18 */
    {.pattern = 0x340, .num_bits = 10}, /* ' ' 19 */
    {.pattern = 0x341, .num_bits = 10}, /* ' ' 20 */
    {.pattern = 0x342, .num_bits = 10}, /* ' ' 21 */
    {.pattern = 0x343, .num_bits = 10}, /* ' ' 22 */
    {.pattern = 0x344, .num_bits = 10}, /* ' ' 23 */
    {.pattern = 0x345, .num_bits = 10}, /* ' ' 24 */
    {.pattern = 0x346, .num_bits = 10}, /* ' ' 25 */
    {.pattern = 0x347, .num_bits = 10}, /* ' ' 26 */
    {.pattern = 0x348, .num_bits = 10}, /* ' ' 27 */
    {.pattern = 0x349, .num_bits = 10}, /* ' ' 28 */
    {.pattern = 0x34a, .num_bits = 10}, /* ' ' 29 */
    {.pattern = 0x34b, .num_bits = 10}, /* ' ' 30 */
    {.pattern = 0x34c, .num_bits = 10}, /* ' ' 31 */
    {.pattern = 0x4, .num_bits = 5},    /* ' ' 32 */
    {.pattern = 0x34d, .num_bits = 10}, /* '!' 33 */
    {.pattern = 0x34e, .num_bits = 10}, /* '"' 34 */
    {.pattern = 0x34f, .num_bits = 10}, /* '#' 35 */
    {.pattern = 0x350, .num_bits = 10}, /* '$' 36 */
    {.pattern = 0x351, .num_bits = 10}, /* '%' 37 */
    {.pattern = 0x352, .num_bits = 10}, /* '&' 38 */
    {.pattern = 0x56, .num_bits = 7},   /* ''' 39 */
    {.pattern = 0x353, .num_bits = 10}, /* '(' 40 */
    {.pattern = 0x354, .num_bits = 10}, /* ')' 41 */
    {.pattern = 0x355, .num_bits = 10}, /* '*' 42 */
    {.pattern = 0x356, .num_bits = 10}, /* '+' 43 */
    {.pattern = 0xb9, .num_bits = 8},   /* ',' 44 */
    {.pattern = 0x188, .num_bits = 9},  /* '-' 45 */
    {.pattern = 0x57, .num_bits = 7},   /* '.' 46 */
    {.pattern = 0x357, .num_bits = 10}, /* '/' 47 */
    {.pattern = 0x358, .num_bits = 10}, /* '0' 48 */
    {.pattern = 0x359, .num_bits = 10}, /* '1' 49 */
    {.pattern = 0x35a, .num_bits = 10}, /* '2' 50 */
    {.pattern = 0x35b, .num_bits = 10}, /* '3' 51 */
    {.pattern = 0x35c, .num_bits = 10}, /* '4' 52 */
    {.pattern = 0x35d, .num_bits = 10}, /* '5' 53 */
    {.pattern = 0x35e, .num_bits = 10}, /* '6' 54 */
    {.pattern = 0x35f, .num_bits = 10}, /* '7' 55 */
    {.pattern = 0x360, .num_bits = 10}, /* '8' 56 */
    {.pattern = 0x361, .num_bits = 10}, /* '9' 57 */
    {.pattern = 0x362, .num_bits = 10}, /* ':' 58 */
    {.pattern = 0x363, .num_bits = 10}, /* ';' 59 */
    {.pattern = 0x364, .num_bits = 10}, /* '<' 60 */
    {.pattern = 0x365, .num_bits = 10}, /* '=' 61 */
    {.pattern = 0x366, .num_bits = 10}, /* '>' 62 */
    {.pattern = 0xba, .num_bits = 8},   /* '?' 63 */
    {.pattern = 0x367, .num_bits = 10}, /* '@' 64 */
    {.pattern = 0x368, .num_bits = 10}, /* 'A' 65 */
    {.pattern = 0xbb, .num_bits = 8},   /* 'B' 66 */
    {.pattern = 0x189, .num_bits = 9},  /* 'C' 67 */
    {.pattern = 0x18a, .num_bits = 9},  /* 'D' 68 */
    {.pattern = 0x18b, .num_bits = 9},  /* 'E' 69 */
    {.pattern = 0x18c, .num_bits = 9},  /* 'F' 70 */
    {.pattern = 0x18d, .num_bits = 9},  /* 'G' 71 */
    {.pattern = 0x18e, .num_bits = 9},  /* 'H' 72 */
    {.pattern = 0xbc, .num_bits = 8},   /* 'I' 73 */
    {.pattern = 0x369, .num_bits = 10}, /* 'J' 74 */
    {.pattern = 0x36a, .num_bits = 10}, /* 'K' 75 */
    {.pattern = 0x18f, .num_bits = 9},  /* 'L' 76 */
    {.pattern = 0x190, .num_bits = 9},  /* 'M' 77 */
    {.pattern = 0x36b, .num_bits = 10}, /* 'N' 78 */
    {.pattern = 0x36c, .num_bits = 10}, /* 'O' 79 */
    {.pattern = 0x191, .num_bits = 9},  /* 'P' 80 */
    {.pattern = 0x36d, .num_bits = 10}, /* 'Q' 81 */
    {.pattern = 0x36e, .num_bits = 10}, /* 'R' 82 */
    {.pattern = 0x36f, .num_bits = 10}, /* 'S' 83 */
    {.pattern = 0xbd, .num_bits = 8},   /* 'T' 84 */
    {.pattern = 0x370, .num_bits = 10}, /* 'U' 85 */
    {.pattern = 0x192, .num_bits = 9},  /* 'V' 86 */
    {.pattern = 0xbe, .num_bits = 8},   /* 'W' 87 */
    {.pattern = 0x371, .num_bits = 10}, /* 'X' 88 */
    {.pattern = 0x193, .num_bits = 9},  /* 'Y' 89 */
    {.pattern = 0x372, .num_bits = 10}, /* 'Z' 90 */
    {.pattern = 0x373, .num_bits = 10}, /* '[' 91 */
    {.pattern = 0x374, .num_bits = 10}, /* '\' 92 */
    {.pattern = 0x375, .num_bits = 10}, /* ']' 93 */
    {.pattern = 0x376, .num_bits = 10}, /* '^' 94 */
    {.pattern = 0x377, .num_bits = 10}, /* '_' 95 */
    {.pattern = 0x378, .num_bits = 10}, /* '`' 96 */
    {.pattern = 0x5, .num_bits = 5},    /* 'a' 97 */
    {.pattern = 0x58, .num_bits = 7},   /* 'b' 98 */
    {.pattern = 0x20, .num_bits = 6},   /* 'c' 99 */
    {.pattern = 0x21, .num_bits = 6},   /* 'd' 100 */
    {.pattern = 0x6, .num_bits = 5},    /* 'e' 101 */
    {.pattern = 0x22, .num_bits = 6},   /* 'f' 102 */
    {.pattern = 0x59, .num_bits = 7},   /* 'g' 103 */
    {.pattern = 0x23, .num_bits = 6},   /* 'h' 104 */
    {.pattern = 0x7, .num_bits = 5},    /* 'i' 105 */
    {.pattern = 0xbf, .num_bits = 8},   /* 'j' 106 */
    {.pattern = 0x24, .num_bits = 6},   /* 'k' 107 */
    {.pattern = 0x25, .num_bits = 6},   /* 'l' 108 */
    {.pattern = 0x26, .num_bits = 6},   /* 'm' 109 */
    {.pattern = 0x8, .num_bits = 5},    /* 'n' 110 */
    {.pattern = 0x9, .num_bits = 5},    /* 'o' 111 */
    {.pattern = 0x5a, .num_bits = 7},   /* 'p' 112 */
    {.pattern = 0x194, .num_bits = 9},  /* 'q' 113 */
    {.pattern = 0xa, .num_bits = 5},    /* 'r' 114 */
    {.pattern = 0xb, .num_bits = 5},    /* 's' 115 */
    {.pattern = 0xc, .num_bits = 5},    /* 't' 116 */
    {.pattern = 0xd, .num_bits = 5},    /* 'u' 117 */
    {.pattern = 0xc0, .num_bits = 8},   /* 'v' 118 */
    {.pattern = 0x27, .num_bits = 6},   /* 'w' 119 */
    {.pattern = 0xc1, .num_bits = 8},   /* 'x' 120 */
    {.pattern = 0x28, .num_bits = 6},   /* 'y' 121 */
    {.pattern = 0x379, .num_bits = 10}, /* 'z' 122 */
    {.pattern = 0x37a, .num_bits = 10}, /* '{' 123 */
    {.pattern = 0x37b, .num_bits = 10}, /* '|' 124 */
    {.pattern = 0x37c, .num_bits = 10}, /* '}' 125 */
    {.pattern = 0x37d, .num_bits = 10}, /* '~' 126 */
    {.pattern = 0x37e, .num_bits = 10}, /* ' ' 127 */
    {.pattern = 0x37f, .num_bits = 10}, /* ' ' 128 */
    {.pattern = 0x380, .num_bits = 10}, /* ' ' 129 */
    {.pattern = 0x381, .num_bits = 10}, /* ' ' 130 */
    {.pattern = 0x382, .num_bits = 10}, /* ' ' 131 */
    {.pattern = 0x383, .num_bits = 10}, /* ' ' 132 */
    {.pattern = 0x384, .num_bits = 10}, /* ' ' 133 */
    {.pattern = 0x385, .num_bits = 10}, /* ' ' 134 */
    {.pattern = 0x386, .num_bits = 10}, /* ' ' 135 */
    {.pattern = 0x387, .num_bits = 10}, /* ' ' 136 */
    {.pattern = 0x388, .num_bits = 10}, /* ' ' 137 */
    {.pattern = 0x389, .num_bits = 10}, /* ' ' 138 */
    {.pattern = 0x38a, .num_bits = 10}, /* ' ' 139 */
    {.pattern = 0x38b, .num_bits = 10}, /* ' ' 140 */
    {.pattern = 0x38c, .num_bits = 10}, /* ' ' 141 */
    {.pattern = 0x38d, .num_bits = 10}, /* ' ' 142 */
    {.pattern = 0x38e, .num_bits = 10}, /* ' ' 143 */
    {.pattern = 0x38f, .num_bits = 10}, /* ' ' 144 */
    {.pattern = 0x390, .num_bits = 10}, /* ' ' 145 */
    {.pattern = 0x391, .num_bits = 10}, /* ' ' 146 */
    {.pattern = 0x392, .num_bits = 10}, /* ' ' 147 */
    {.pattern = 0x393, .num_bits = 10}, /* ' ' 148 */
    {.pattern = 0x394, .num_bits = 10}, /* ' ' 149 */
    {.pattern = 0x395, .num_bits = 10}, /* ' ' 150 */
    {.pattern = 0x396, .num_bits = 10}, /* ' ' 151 */
    {.pattern = 0x397, .num_bits = 10}, /* ' ' 152 */
    {.pattern = 0x398, .num_bits = 10}, /* ' ' 153 */
    {.pattern = 0x399, .num_bits = 10}, /* ' ' 154 */
    {.pattern = 0x39a, .num_bits = 10}, /* ' ' 155 */
    {.pattern = 0x39b, .num_bits = 10}, /* ' ' 156 */
    {.pattern = 0x39c, .num_bits = 10}, /* ' ' 157 */
    {.pattern = 0x39d, .num_bits = 10}, /* ' ' 158 */
    {.pattern = 0x39e, .num_bits = 10}, /* ' ' 159 */
    {.pattern = 0x39f, .num_bits = 10}, /* ' ' 160 */
    {.pattern = 0x3a0, .num_bits = 10}, /* ' ' 161 */
    {.pattern = 0x3a1, .num_bits = 10}, /* ' ' 162 */
    {.pattern = 0x3a2, .num_bits = 10}, /* ' ' 163 */
    {.pattern = 0x3a3, .num_bits = 10}, /* ' ' 164 */
    {.pattern = 0x3a4, .num_bits = 10}, /* ' ' 165 */
    {.pattern = 0x3a5, .num_bits = 10}, /* ' ' 166 */
    {.pattern = 0x3a6, .num_bits = 10}, /* ' ' 167 */
    {.pattern = 0x3a7, .num_bits = 10}, /* ' ' 168 */
    {.pattern = 0x3a8, .num_bits = 10}, /* ' ' 169 */
    {.pattern = 0x3a9, .num_bits = 10}, /* ' ' 170 */
    {.pattern = 0x3aa, .num_bits = 10}, /* ' ' 171 */
    {.pattern = 0x3ab, .num_bits = 10}, /* ' ' 172 */
    {.pattern = 0x3ac, .num_bits = 10}, /* ' ' 173 */
    {.pattern = 0x3ad, .num_bits = 10}, /* ' ' 174 */
    {.pattern = 0x3ae, .num_bits = 10}, /* ' ' 175 */
    {.pattern = 0x3af, .num_bits = 10}, /* ' ' 176 */
    {.pattern = 0x3b0, .num_bits = 10}, /* ' ' 177 */
    {.pattern = 0x3b1, .num_bits = 10}, /* ' ' 178 */
    {.pattern = 0x3b2, .num_bits = 10}, /* ' ' 179 */
    {.pattern = 0x3b3, .num_bits = 10}, /* ' ' 180 */
    {.pattern = 0x3b4, .num_bits = 10}, /* ' ' 181 */
    {.pattern = 0x3b5, .num_bits = 10}, /* ' ' 182 */
    {.pattern = 0x3b6, .num_bits = 10}, /* ' ' 183 */
    {.pattern = 0x3b7, .num_bits = 10}, /* ' ' 184 */
    {.pattern = 0x3b8, .num_bits = 10}, /* ' ' 185 */
    {.pattern = 0x3b9, .num_bits = 10}, /* ' ' 186 */
    {.pattern = 0x3ba, .num_bits = 10}, /* ' ' 187 */
    {.pattern = 0x3bb, .num_bits = 10}, /* ' ' 188 */
    {.pattern = 0x3bc, .num_bits = 10}, /* ' ' 189 */
    {.pattern = 0x3bd, .num_bits = 10}, /* ' ' 190 */
    {.pattern = 0x3be, .num_bits = 10}, /* ' ' 191 */
    {.pattern = 0x3bf, .num_bits = 10}, /* ' ' 192 */
    {.pattern = 0x3c0, .num_bits = 10}, /* ' ' 193 */
    {.pattern = 0x3c1, .num_bits = 10}, /* ' ' 194 */
    {.pattern = 0x3c2, .num_bits = 10}, /* ' ' 195 */
    {.pattern = 0x3c3, .num_bits = 10}, /* ' ' 196 */
    {.pattern = 0x3c4, .num_bits = 10}, /* ' ' 197 */
    {.pattern = 0x3c5, .num_bits = 10}, /* ' ' 198 */
    {.pattern = 0x3c6, .num_bits = 10}, /* ' ' 199 */
    {.pattern = 0x3c7, .num_bits = 10}, /* ' ' 200 */
    {.pattern = 0x3c8, .num_bits = 10}, /* ' ' 201 */
    {.pattern = 0x3c9, .num_bits = 10}, /* ' ' 202 */
    {.pattern = 0x3ca, .num_bits = 10}, /* ' ' 203 */
    {.pattern = 0x3cb, .num_bits = 10}, /* ' ' 204 */
    {.pattern = 0x3cc, .num_bits = 10}, /* ' ' 205 */
    {.pattern = 0x3cd, .num_bits = 10}, /* ' ' 206 */
    {.pattern = 0x3ce, .num_bits = 10}, /* ' ' 207 */
    {.pattern = 0x3cf, .num_bits = 10}, /* ' ' 208 */
    {.pattern = 0x3d0, .num_bits = 10}, /* ' ' 209 */
    {.pattern = 0x3d1, .num_bits = 10}, /* ' ' 210 */
    {.pattern = 0x3d2, .num_bits = 10}, /* ' ' 211 */
    {.pattern = 0x3d3, .num_bits = 10}, /* ' ' 212 */
    {.pattern = 0x3d4, .num_bits = 10}, /* ' ' 213 */
    {.pattern = 0x3d5, .num_bits = 10}, /* ' ' 214 */
    {.pattern = 0x3d6, .num_bits = 10}, /* ' ' 215 */
    {.pattern = 0x3d7, .num_bits = 10}, /* ' ' 216 */
    {.pattern = 0x3d8, .num_bits = 10}, /* ' ' 217 */
    {.pattern = 0x3d9, .num_bits = 10}, /* ' ' 218 */
    {.pattern = 0x3da, .num_bits = 10}, /* ' ' 219 */
    {.pattern = 0x3db, .num_bits = 10}, /* ' ' 220 */
    {.pattern = 0x3dc, .num_bits = 10}, /* ' ' 221 */
    {.pattern = 0x3dd, .num_bits = 10}, /* ' ' 222 */
    {.pattern = 0x3de, .num_bits = 10}, /* ' ' 223 */
    {.pattern = 0x3df, .num_bits = 10}, /* ' ' 224 */
    {.pattern = 0x3e0, .num_bits = 10}, /* ' ' 225 */
    {.pattern = 0x3e1, .num_bits = 10}, /* ' ' 226 */
    {.pattern = 0x3e2, .num_bits = 10}, /* ' ' 227 */
    {.pattern = 0x3e3, .num_bits = 10}, /* ' ' 228 */
    {.pattern = 0x3e4, .num_bits = 10}, /* ' ' 229 */
    {.pattern = 0x3e5, .num_bits = 10}, /* ' ' 230 */
    {.pattern = 0x3e6, .num_bits = 10}, /* ' ' 231 */
    {.pattern = 0x3e7, .num_bits = 10}, /* ' ' 232 */
    {.pattern = 0x3e8, .num_bits = 10}, /* ' ' 233 */
    {.pattern = 0x3e9, .num_bits = 10}, /* ' ' 234 */
    {.pattern = 0x3ea, .num_bits = 10}, /* ' ' 235 */
    {.pattern = 0x3eb, .num_bits = 10}, /* ' ' 236 */
    {.pattern = 0x3ec, .num_bits = 10}, /* ' ' 237 */
    {.pattern = 0x3ed, .num_bits = 10}, /* ' ' 238 */
    {.pattern = 0x3ee, .num_bits = 10}, /* ' ' 239 */
    {.pattern = 0x3ef, .num_bits = 10}, /* ' ' 240 */
    {.pattern = 0x3f0, .num_bits = 10}, /* ' ' 241 */
    {.pattern = 0x3f1, .num_bits = 10}, /* ' ' 242 */
    {.pattern = 0x3f2, .num_bits = 10}, /* ' ' 243 */
    {.pattern = 0x3f3, .num_bits = 10}, /* ' ' 244 */
    {.pattern = 0x3f4, .num_bits = 10}, /* ' ' 245 */
    {.pattern = 0x3f5, .num_bits = 10}, /* ' ' 246 */
    {.pattern = 0x3f6, .num_bits = 10}, /* ' ' 247 */
    {.pattern = 0x3f7, .num_bits = 10}, /* ' ' 248 */
    {.pattern = 0x3f8, .num_bits = 10}, /* ' ' 249 */
    {.pattern = 0x3f9, .num_bits = 10}, /* ' ' 250 */
    {.pattern = 0x3fa, .num_bits = 10}, /* ' ' 251 */
    {.pattern = 0x3fb, .num_bits = 10}, /* ' ' 252 */
    {.pattern = 0x3fc, .num_bits = 10}, /* ' ' 253 */
    {.pattern = 0x3fd, .num_bits = 10}, /* ' ' 254 */
    {.pattern = 0x3fe, .num_bits = 10}, /* ' ' 255 */
};

static struct aws_huffman_code encode_symbol(uint8_t symbol, void *userdata) {
    (void)userdata;

    return code_points[symbol];
}

/* NOLINTNEXTLINE(readability-function-size) */
static uint8_t decode_symbol(uint32_t bits, uint8_t *symbol, void *userdata) {
    (void)userdata;

    if (bits & 0x80000000) {
        goto node_1;
    } else {
        goto node_0;
    }

node_0:
    if (bits & 0x40000000) {
        goto node_01;
    } else {
        goto node_00;
    }

node_00:
    if (bits & 0x20000000) {
        goto node_001;
    } else {
        return 0; /* invalid node */
    }

node_001:
    if (bits & 0x10000000) {
        goto node_0011;
    } else {
        goto node_0010;
    }

node_0010:
    if (bits & 0x8000000) {
        *symbol = 97;
        return 5;
    } else {
        *symbol = 32;
        return 5;
    }

node_0011:
    if (bits & 0x8000000) {
        *symbol = 105;
        return 5;
    } else {
        *symbol = 101;
        return 5;
    }

node_01:
    if (bits & 0x20000000) {
        goto node_011;
    } else {
        goto node_010;
    }

node_010:
    if (bits & 0x10000000) {
        goto node_0101;
    } else {
        goto node_0100;
    }

node_0100:
    if (bits & 0x8000000) {
        *symbol = 111;
        return 5;
    } else {
        *symbol = 110;
        return 5;
    }

node_0101:
    if (bits & 0x8000000) {
        *symbol = 115;
        return 5;
    } else {
        *symbol = 114;
        return 5;
    }

node_011:
    if (bits & 0x10000000) {
        return 0; /* invalid node */
    } else {
        goto node_0110;
    }

node_0110:
    if (bits & 0x8000000) {
        *symbol = 117;
        return 5;
    } else {
        *symbol = 116;
        return 5;
    }

node_1:
    if (bits & 0x40000000) {
        goto node_11;
    } else {
        goto node_10;
    }

node_10:
    if (bits & 0x20000000) {
        goto node_101;
    } else {
        goto node_100;
    }

node_100:
    if (bits & 0x10000000) {
        goto node_1001;
    } else {
        goto node_1000;
    }

node_1000:
    if (bits & 0x8000000) {
        goto node_10001;
    } else {
        goto node_10000;
    }

node_10000:
    if (bits & 0x4000000) {
        *symbol = 100;
        return 6;
    } else {
        *symbol = 99;
        return 6;
    }

node_10001:
    if (bits & 0x4000000) {
        *symbol = 104;
        return 6;
    } else {
        *symbol = 102;
        return 6;
    }

node_1001:
    if (bits & 0x8000000) {
        goto node_10011;
    } else {
        goto node_10010;
    }

node_10010:
    if (bits & 0x4000000) {
        *symbol = 108;
        return 6;
    } else {
        *symbol = 107;
        return 6;
    }

node_10011:
    if (bits & 0x4000000) {
        *symbol = 119;
        return 6;
    } else {
        *symbol = 109;
        return 6;
    }

node_101:
    if (bits & 0x10000000) {
        goto node_1011;
    } else {
        goto node_1010;
    }

node_1010:
    if (bits & 0x8000000) {
        goto node_10101;
    } else {
        goto node_10100;
    }

node_10100:
    if (bits & 0x4000000) {
        return 0; /* invalid node */
    } else {
        *symbol = 121;
        return 6;
    }

node_10101:
    if (bits & 0x4000000) {
        goto node_101011;
    } else {
        return 0; /* invalid node */
    }

node_101011:
    if (bits & 0x2000000) {
        *symbol = 46;
        return 7;
    } else {
        *symbol = 39;
        return 7;
    }

node_1011:
    if (bits & 0x8000000) {
        goto node_10111;
    } else {
        goto node_10110;
    }

node_10110:
    if (bits & 0x4000000) {
        goto node_101101;
    } else {
        goto node_101100;
    }

node_101100:
    if (bits & 0x2000000) {
        *symbol = 103;
        return 7;
    } else {
        *symbol = 98;
        return 7;
    }

node_101101:
    if (bits & 0x2000000) {
        return 0; /* invalid node */
    } else {
        *symbol = 112;
        return 7;
    }

node_10111:
    if (bits & 0x4000000) {
        goto node_101111;
    } else {
        goto node_101110;
    }

node_101110:
    if (bits & 0x2000000) {
        goto node_1011101;
    } else {
        goto node_1011100;
    }

node_1011100:
    if (bits & 0x1000000) {
        *symbol = 44;
        return 8;
    } else {
        *symbol = 10;
        return 8;
    }

node_1011101:
    if (bits & 0x1000000) {
        *symbol = 66;
        return 8;
    } else {
        *symbol = 63;
        return 8;
    }

node_101111:
    if (bits & 0x2000000) {
        goto node_1011111;
    } else {
        goto node_1011110;
    }

node_1011110:
    if (bits & 0x1000000) {
        *symbol = 84;
        return 8;
    } else {
        *symbol = 73;
        return 8;
    }

node_1011111:
    if (bits & 0x1000000) {
        *symbol = 106;
        return 8;
    } else {
        *symbol = 87;
        return 8;
    }

node_11:
    if (bits & 0x20000000) {
        goto node_111;
    } else {
        goto node_110;
    }

node_110:
    if (bits & 0x10000000) {
        goto node_1101;
    } else {
        goto node_1100;
    }

node_1100:
    if (bits & 0x8000000) {
        goto node_11001;
    } else {
        goto node_11000;
    }

node_11000:
    if (bits & 0x4000000) {
        goto node_110001;
    } else {
        goto node_110000;
    }

node_110000:
    if (bits & 0x2000000) {
        return 0; /* invalid node */
    } else {
        goto node_1100000;
    }

node_1100000:
    if (bits & 0x1000000) {
        *symbol = 120;
        return 8;
    } else {
        *symbol = 118;
        return 8;
    }

node_110001:
    if (bits & 0x2000000) {
        goto node_1100011;
    } else {
        goto node_1100010;
    }

node_1100010:
    if (bits & 0x1000000) {
        goto node_11000101;
    } else {
        goto node_11000100;
    }

node_11000100:
    if (bits & 0x800000) {
        *symbol = 67;
        return 9;
    } else {
        *symbol = 45;
        return 9;
    }

node_11000101:
    if (bits & 0x800000) {
        *symbol = 69;
        return 9;
    } else {
        *symbol = 68;
        return 9;
    }

node_1100011:
    if (bits & 0x1000000) {
        goto node_11000111;
    } else {
        goto node_11000110;
    }

node_11000110:
    if (bits & 0x800000) {
        *symbol = 71;
        return 9;
    } else {
        *symbol = 70;
        return 9;
    }

node_11000111:
    if (bits & 0x800000) {
        *symbol = 76;
        return 9;
    } else {
        *symbol = 72;
        return 9;
    }

node_11001:
    if (bits & 0x4000000) {
        goto node_110011;
    } else {
        goto node_110010;
    }

node_110010:
    if (bits & 0x2000000) {
        goto node_1100101;
    } else {
        goto node_1100100;
    }

node_1100100:
    if (bits & 0x1000000) {
        goto node_11001001;
    } else {
        goto node_11001000;
    }

node_11001000:
    if (bits & 0x800000) {
        *symbol = 80;
        return 9;
    } else {
        *symbol = 77;
        return 9;
    }

node_11001001:
    if (bits & 0x800000) {
        *symbol = 89;
        return 9;
    } else {
        *symbol = 86;
        return 9;
    }

node_1100101:
    if (bits & 0x1000000) {
        goto node_11001011;
    } else {
        goto node_11001010;
    }

node_11001010:
    if (bits & 0x800000) {
        return 0; /* invalid node */
    } else {
        *symbol = 113;
        return 9;
    }

node_11001011:
    if (bits & 0x800000) {
        goto node_110010111;
    } else {
        return 0; /* invalid node */
    }

node_110010111:
    if (bits & 0x400000) {
        *symbol = 1;
        return 10;
    } else {
        *symbol = 0;
        return 10;
    }

node_110011:
    if (bits & 0x2000000) {
        goto node_1100111;
    } else {
        goto node_1100110;
    }

node_1100110:
    if (bits & 0x1000000) {
        goto node_11001101;
    } else {
        goto node_11001100;
    }

node_11001100:
    if (bits & 0x800000) {
        goto node_110011001;
    } else {
        goto node_110011000;
    }

node_110011000:
    if (bits & 0x400000) {
        *symbol = 3;
        return 10;
    } else {
        *symbol = 2;
        return 10;
    }

node_110011001:
    if (bits & 0x400000) {
        *symbol = 5;
        return 10;
    } else {
        *symbol = 4;
        return 10;
    }

node_11001101:
    if (bits & 0x800000) {
        goto node_110011011;
    } else {
        goto node_110011010;
    }

node_110011010:
    if (bits & 0x400000) {
        *symbol = 7;
        return 10;
    } else {
        *symbol = 6;
        return 10;
    }

node_110011011:
    if (bits & 0x400000) {
        *symbol = 9;
        return 10;
    } else {
        *symbol = 8;
        return 10;
    }

node_1100111:
    if (bits & 0x1000000) {
        goto node_11001111;
    } else {
        goto node_11001110;
    }

node_11001110:
    if (bits & 0x800000) {
        goto node_110011101;
    } else {
        goto node_110011100;
    }

node_110011100:
    if (bits & 0x400000) {
        *symbol = 12;
        return 10;
    } else {
        *symbol = 11;
        return 10;
    }

node_110011101:
    if (bits & 0x400000) {
        *symbol = 14;
        return 10;
    } else {
        *symbol = 13;
        return 10;
    }

node_11001111:
    if (bits & 0x800000) {
        goto node_110011111;
    } else {
        goto node_110011110;
    }

node_110011110:
    if (bits & 0x400000) {
        *symbol = 16;
        return 10;
    } else {
        *symbol = 15;
        return 10;
    }

node_110011111:
    if (bits & 0x400000) {
        *symbol = 18;
        return 10;
    } else {
        *symbol = 17;
        return 10;
    }

node_1101:
    if (bits & 0x8000000) {
        goto node_11011;
    } else {
        goto node_11010;
    }

node_11010:
    if (bits & 0x4000000) {
        goto node_110101;
    } else {
        goto node_110100;
    }

node_110100:
    if (bits & 0x2000000) {
        goto node_1101001;
    } else {
        goto node_1101000;
    }

node_1101000:
    if (bits & 0x1000000) {
        goto node_11010001;
    } else {
        goto node_11010000;
    }

node_11010000:
    if (bits & 0x800000) {
        goto node_110100001;
    } else {
        goto node_110100000;
    }

node_110100000:
    if (bits & 0x400000) {
        *symbol = 20;
        return 10;
    } else {
        *symbol = 19;
        return 10;
    }

node_110100001:
    if (bits & 0x400000) {
        *symbol = 22;
        return 10;
    } else {
        *symbol = 21;
        return 10;
    }

node_11010001:
    if (bits & 0x800000) {
        goto node_110100011;
    } else {
        goto node_110100010;
    }

node_110100010:
    if (bits & 0x400000) {
        *symbol = 24;
        return 10;
    } else {
        *symbol = 23;
        return 10;
    }

node_110100011:
    if (bits & 0x400000) {
        *symbol = 26;
        return 10;
    } else {
        *symbol = 25;
        return 10;
    }

node_1101001:
    if (bits & 0x1000000) {
        goto node_11010011;
    } else {
        goto node_11010010;
    }

node_11010010:
    if (bits & 0x800000) {
        goto node_110100101;
    } else {
        goto node_110100100;
    }

node_110100100:
    if (bits & 0x400000) {
        *symbol = 28;
        return 10;
    } else {
        *symbol = 27;
        return 10;
    }

node_110100101:
    if (bits & 0x400000) {
        *symbol = 30;
        return 10;
    } else {
        *symbol = 29;
        return 10;
    }

node_11010011:
    if (bits & 0x800000) {
        goto node_110100111;
    } else {
        goto node_110100110;
    }

node_110100110:
    if (bits & 0x400000) {
        *symbol = 33;
        return 10;
    } else {
        *symbol = 31;
        return 10;
    }

node_110100111:
    if (bits & 0x400000) {
        *symbol = 35;
        return 10;
    } else {
        *symbol = 34;
        return 10;
    }

node_110101:
    if (bits & 0x2000000) {
        goto node_1101011;
    } else {
        goto node_1101010;
    }

node_1101010:
    if (bits & 0x1000000) {
        goto node_11010101;
    } else {
        goto node_11010100;
    }

node_11010100:
    if (bits & 0x800000) {
        goto node_110101001;
    } else {
        goto node_110101000;
    }

node_110101000:
    if (bits & 0x400000) {
        *symbol = 37;
        return 10;
    } else {
        *symbol = 36;
        return 10;
    }

node_110101001:
    if (bits & 0x400000) {
        *symbol = 40;
        return 10;
    } else {
        *symbol = 38;
        return 10;
    }

node_11010101:
    if (bits & 0x800000) {
        goto node_110101011;
    } else {
        goto node_110101010;
    }

node_110101010:
    if (bits & 0x400000) {
        *symbol = 42;
        return 10;
    } else {
        *symbol = 41;
        return 10;
    }

node_110101011:
    if (bits & 0x400000) {
        *symbol = 47;
        return 10;
    } else {
        *symbol = 43;
        return 10;
    }

node_1101011:
    if (bits & 0x1000000) {
        goto node_11010111;
    } else {
        goto node_11010110;
    }

node_11010110:
    if (bits & 0x800000) {
        goto node_110101101;
    } else {
        goto node_110101100;
    }

node_110101100:
    if (bits & 0x400000) {
        *symbol = 49;
        return 10;
    } else {
        *symbol = 48;
        return 10;
    }

node_110101101:
    if (bits & 0x400000) {
        *symbol = 51;
        return 10;
    } else {
        *symbol = 50;
        return 10;
    }

node_11010111:
    if (bits & 0x800000) {
        goto node_110101111;
    } else {
        goto node_110101110;
    }

node_110101110:
    if (bits & 0x400000) {
        *symbol = 53;
        return 10;
    } else {
        *symbol = 52;
        return 10;
    }

node_110101111:
    if (bits & 0x400000) {
        *symbol = 55;
        return 10;
    } else {
        *symbol = 54;
        return 10;
    }

node_11011:
    if (bits & 0x4000000) {
        goto node_110111;
    } else {
        goto node_110110;
    }

node_110110:
    if (bits & 0x2000000) {
        goto node_1101101;
    } else {
        goto node_1101100;
    }

node_1101100:
    if (bits & 0x1000000) {
        goto node_11011001;
    } else {
        goto node_11011000;
    }

node_11011000:
    if (bits & 0x800000) {
        goto node_110110001;
    } else {
        goto node_110110000;
    }

node_110110000:
    if (bits & 0x400000) {
        *symbol = 57;
        return 10;
    } else {
        *symbol = 56;
        return 10;
    }

node_110110001:
    if (bits & 0x400000) {
        *symbol = 59;
        return 10;
    } else {
        *symbol = 58;
        return 10;
    }

node_11011001:
    if (bits & 0x800000) {
        goto node_110110011;
    } else {
        goto node_110110010;
    }

node_110110010:
    if (bits & 0x400000) {
        *symbol = 61;
        return 10;
    } else {
        *symbol = 60;
        return 10;
    }

node_110110011:
    if (bits & 0x400000) {
        *symbol = 64;
        return 10;
    } else {
        *symbol = 62;
        return 10;
    }

node_1101101:
    if (bits & 0x1000000) {
        goto node_11011011;
    } else {
        goto node_11011010;
    }

node_11011010:
    if (bits & 0x800000) {
        goto node_110110101;
    } else {
        goto node_110110100;
    }

node_110110100:
    if (bits & 0x400000) {
        *symbol = 74;
        return 10;
    } else {
        *symbol = 65;
        return 10;
    }

node_110110101:
    if (bits & 0x400000) {
        *symbol = 78;
        return 10;
    } else {
        *symbol = 75;
        return 10;
    }

node_11011011:
    if (bits & 0x800000) {
        goto node_110110111;
    } else {
        goto node_110110110;
    }

node_110110110:
    if (bits & 0x400000) {
        *symbol = 81;
        return 10;
    } else {
        *symbol = 79;
        return 10;
    }

node_110110111:
    if (bits & 0x400000) {
        *symbol = 83;
        return 10;
    } else {
        *symbol = 82;
        return 10;
    }

node_110111:
    if (bits & 0x2000000) {
        goto node_1101111;
    } else {
        goto node_1101110;
    }

node_1101110:
    if (bits & 0x1000000) {
        goto node_11011101;
    } else {
        goto node_11011100;
    }

node_11011100:
    if (bits & 0x800000) {
        goto node_110111001;
    } else {
        goto node_110111000;
    }

node_110111000:
    if (bits & 0x400000) {
        *symbol = 88;
        return 10;
    } else {
        *symbol = 85;
        return 10;
    }

node_110111001:
    if (bits & 0x400000) {
        *symbol = 91;
        return 10;
    } else {
        *symbol = 90;
        return 10;
    }

node_11011101:
    if (bits & 0x800000) {
        goto node_110111011;
    } else {
        goto node_110111010;
    }

node_110111010:
    if (bits & 0x400000) {
        *symbol = 93;
        return 10;
    } else {
        *symbol = 92;
        return 10;
    }

node_110111011:
    if (bits & 0x400000) {
        *symbol = 95;
        return 10;
    } else {
        *symbol = 94;
        return 10;
    }

node_1101111:
    if (bits & 0x1000000) {
        goto node_11011111;
    } else {
        goto node_11011110;
    }

node_11011110:
    if (bits & 0x800000) {
        goto node_110111101;
    } else {
        goto node_110111100;
    }

node_110111100:
    if (bits & 0x400000) {
        *symbol = 122;
        return 10;
    } else {
        *symbol = 96;
        return 10;
    }

node_110111101:
    if (bits & 0x400000) {
        *symbol = 124;
        return 10;
    } else {
        *symbol = 123;
        return 10;
    }

node_11011111:
    if (bits & 0x800000) {
        goto node_110111111;
    } else {
        goto node_110111110;
    }

node_110111110:
    if (bits & 0x400000) {
        *symbol = 126;
        return 10;
    } else {
        *symbol = 125;
        return 10;
    }

node_110111111:
    if (bits & 0x400000) {
        *symbol = 128;
        return 10;
    } else {
        *symbol = 127;
        return 10;
    }

node_111:
    if (bits & 0x10000000) {
        goto node_1111;
    } else {
        goto node_1110;
    }

node_1110:
    if (bits & 0x8000000) {
        goto node_11101;
    } else {
        goto node_11100;
    }

node_11100:
    if (bits & 0x4000000) {
        goto node_111001;
    } else {
        goto node_111000;
    }

node_111000:
    if (bits & 0x2000000) {
        goto node_1110001;
    } else {
        goto node_1110000;
    }

node_1110000:
    if (bits & 0x1000000) {
        goto node_11100001;
    } else {
        goto node_11100000;
    }

node_11100000:
    if (bits & 0x800000) {
        goto node_111000001;
    } else {
        goto node_111000000;
    }

node_111000000:
    if (bits & 0x400000) {
        *symbol = 130;
        return 10;
    } else {
        *symbol = 129;
        return 10;
    }

node_111000001:
    if (bits & 0x400000) {
        *symbol = 132;
        return 10;
    } else {
        *symbol = 131;
        return 10;
    }

node_11100001:
    if (bits & 0x800000) {
        goto node_111000011;
    } else {
        goto node_111000010;
    }

node_111000010:
    if (bits & 0x400000) {
        *symbol = 134;
        return 10;
    } else {
        *symbol = 133;
        return 10;
    }

node_111000011:
    if (bits & 0x400000) {
        *symbol = 136;
        return 10;
    } else {
        *symbol = 135;
        return 10;
    }

node_1110001:
    if (bits & 0x1000000) {
        goto node_11100011;
    } else {
        goto node_11100010;
    }

node_11100010:
    if (bits & 0x800000) {
        goto node_111000101;
    } else {
        goto node_111000100;
    }

node_111000100:
    if (bits & 0x400000) {
        *symbol = 138;
        return 10;
    } else {
        *symbol = 137;
        return 10;
    }

node_111000101:
    if (bits & 0x400000) {
        *symbol = 140;
        return 10;
    } else {
        *symbol = 139;
        return 10;
    }

node_11100011:
    if (bits & 0x800000) {
        goto node_111000111;
    } else {
        goto node_111000110;
    }

node_111000110:
    if (bits & 0x400000) {
        *symbol = 142;
        return 10;
    } else {
        *symbol = 141;
        return 10;
    }

node_111000111:
    if (bits & 0x400000) {
        *symbol = 144;
        return 10;
    } else {
        *symbol = 143;
        return 10;
    }

node_111001:
    if (bits & 0x2000000) {
        goto node_1110011;
    } else {
        goto node_1110010;
    }

node_1110010:
    if (bits & 0x1000000) {
        goto node_11100101;
    } else {
        goto node_11100100;
    }

node_11100100:
    if (bits & 0x800000) {
        goto node_111001001;
    } else {
        goto node_111001000;
    }

node_111001000:
    if (bits & 0x400000) {
        *symbol = 146;
        return 10;
    } else {
        *symbol = 145;
        return 10;
    }

node_111001001:
    if (bits & 0x400000) {
        *symbol = 148;
        return 10;
    } else {
        *symbol = 147;
        return 10;
    }

node_11100101:
    if (bits & 0x800000) {
        goto node_111001011;
    } else {
        goto node_111001010;
    }

node_111001010:
    if (bits & 0x400000) {
        *symbol = 150;
        return 10;
    } else {
        *symbol = 149;
        return 10;
    }

node_111001011:
    if (bits & 0x400000) {
        *symbol = 152;
        return 10;
    } else {
        *symbol = 151;
        return 10;
    }

node_1110011:
    if (bits & 0x1000000) {
        goto node_11100111;
    } else {
        goto node_11100110;
    }

node_11100110:
    if (bits & 0x800000) {
        goto node_111001101;
    } else {
        goto node_111001100;
    }

node_111001100:
    if (bits & 0x400000) {
        *symbol = 154;
        return 10;
    } else {
        *symbol = 153;
        return 10;
    }

node_111001101:
    if (bits & 0x400000) {
        *symbol = 156;
        return 10;
    } else {
        *symbol = 155;
        return 10;
    }

node_11100111:
    if (bits & 0x800000) {
        goto node_111001111;
    } else {
        goto node_111001110;
    }

node_111001110:
    if (bits & 0x400000) {
        *symbol = 158;
        return 10;
    } else {
        *symbol = 157;
        return 10;
    }

node_111001111:
    if (bits & 0x400000) {
        *symbol = 160;
        return 10;
    } else {
        *symbol = 159;
        return 10;
    }

node_11101:
    if (bits & 0x4000000) {
        goto node_111011;
    } else {
        goto node_111010;
    }

node_111010:
    if (bits & 0x2000000) {
        goto node_1110101;
    } else {
        goto node_1110100;
    }

node_1110100:
    if (bits & 0x1000000) {
        goto node_11101001;
    } else {
        goto node_11101000;
    }

node_11101000:
    if (bits & 0x800000) {
        goto node_111010001;
    } else {
        goto node_111010000;
    }

node_111010000:
    if (bits & 0x400000) {
        *symbol = 162;
        return 10;
    } else {
        *symbol = 161;
        return 10;
    }

node_111010001:
    if (bits & 0x400000) {
        *symbol = 164;
        return 10;
    } else {
        *symbol = 163;
        return 10;
    }

node_11101001:
    if (bits & 0x800000) {
        goto node_111010011;
    } else {
        goto node_111010010;
    }

node_111010010:
    if (bits & 0x400000) {
        *symbol = 166;
        return 10;
    } else {
        *symbol = 165;
        return 10;
    }

node_111010011:
    if (bits & 0x400000) {
        *symbol = 168;
        return 10;
    } else {
        *symbol = 167;
        return 10;
    }

node_1110101:
    if (bits & 0x1000000) {
        goto node_11101011;
    } else {
        goto node_11101010;
    }

node_11101010:
    if (bits & 0x800000) {
        goto node_111010101;
    } else {
        goto node_111010100;
    }

node_111010100:
    if (bits & 0x400000) {
        *symbol = 170;
        return 10;
    } else {
        *symbol = 169;
        return 10;
    }

node_111010101:
    if (bits & 0x400000) {
        *symbol = 172;
        return 10;
    } else {
        *symbol = 171;
        return 10;
    }

node_11101011:
    if (bits & 0x800000) {
        goto node_111010111;
    } else {
        goto node_111010110;
    }

node_111010110:
    if (bits & 0x400000) {
        *symbol = 174;
        return 10;
    } else {
        *symbol = 173;
        return 10;
    }

node_111010111:
    if (bits & 0x400000) {
        *symbol = 176;
        return 10;
    } else {
        *symbol = 175;
        return 10;
    }

node_111011:
    if (bits & 0x2000000) {
        goto node_1110111;
    } else {
        goto node_1110110;
    }

node_1110110:
    if (bits & 0x1000000) {
        goto node_11101101;
    } else {
        goto node_11101100;
    }

node_11101100:
    if (bits & 0x800000) {
        goto node_111011001;
    } else {
        goto node_111011000;
    }

node_111011000:
    if (bits & 0x400000) {
        *symbol = 178;
        return 10;
    } else {
        *symbol = 177;
        return 10;
    }

node_111011001:
    if (bits & 0x400000) {
        *symbol = 180;
        return 10;
    } else {
        *symbol = 179;
        return 10;
    }

node_11101101:
    if (bits & 0x800000) {
        goto node_111011011;
    } else {
        goto node_111011010;
    }

node_111011010:
    if (bits & 0x400000) {
        *symbol = 182;
        return 10;
    } else {
        *symbol = 181;
        return 10;
    }

node_111011011:
    if (bits & 0x400000) {
        *symbol = 184;
        return 10;
    } else {
        *symbol = 183;
        return 10;
    }

node_1110111:
    if (bits & 0x1000000) {
        goto node_11101111;
    } else {
        goto node_11101110;
    }

node_11101110:
    if (bits & 0x800000) {
        goto node_111011101;
    } else {
        goto node_111011100;
    }

node_111011100:
    if (bits & 0x400000) {
        *symbol = 186;
        return 10;
    } else {
        *symbol = 185;
        return 10;
    }

node_111011101:
    if (bits & 0x400000) {
        *symbol = 188;
        return 10;
    } else {
        *symbol = 187;
        return 10;
    }

node_11101111:
    if (bits & 0x800000) {
        goto node_111011111;
    } else {
        goto node_111011110;
    }

node_111011110:
    if (bits & 0x400000) {
        *symbol = 190;
        return 10;
    } else {
        *symbol = 189;
        return 10;
    }

node_111011111:
    if (bits & 0x400000) {
        *symbol = 192;
        return 10;
    } else {
        *symbol = 191;
        return 10;
    }

node_1111:
    if (bits & 0x8000000) {
        goto node_11111;
    } else {
        goto node_11110;
    }

node_11110:
    if (bits & 0x4000000) {
        goto node_111101;
    } else {
        goto node_111100;
    }

node_111100:
    if (bits & 0x2000000) {
        goto node_1111001;
    } else {
        goto node_1111000;
    }

node_1111000:
    if (bits & 0x1000000) {
        goto node_11110001;
    } else {
        goto node_11110000;
    }

node_11110000:
    if (bits & 0x800000) {
        goto node_111100001;
    } else {
        goto node_111100000;
    }

node_111100000:
    if (bits & 0x400000) {
        *symbol = 194;
        return 10;
    } else {
        *symbol = 193;
        return 10;
    }

node_111100001:
    if (bits & 0x400000) {
        *symbol = 196;
        return 10;
    } else {
        *symbol = 195;
        return 10;
    }

node_11110001:
    if (bits & 0x800000) {
        goto node_111100011;
    } else {
        goto node_111100010;
    }

node_111100010:
    if (bits & 0x400000) {
        *symbol = 198;
        return 10;
    } else {
        *symbol = 197;
        return 10;
    }

node_111100011:
    if (bits & 0x400000) {
        *symbol = 200;
        return 10;
    } else {
        *symbol = 199;
        return 10;
    }

node_1111001:
    if (bits & 0x1000000) {
        goto node_11110011;
    } else {
        goto node_11110010;
    }

node_11110010:
    if (bits & 0x800000) {
        goto node_111100101;
    } else {
        goto node_111100100;
    }

node_111100100:
    if (bits & 0x400000) {
        *symbol = 202;
        return 10;
    } else {
        *symbol = 201;
        return 10;
    }

node_111100101:
    if (bits & 0x400000) {
        *symbol = 204;
        return 10;
    } else {
        *symbol = 203;
        return 10;
    }

node_11110011:
    if (bits & 0x800000) {
        goto node_111100111;
    } else {
        goto node_111100110;
    }

node_111100110:
    if (bits & 0x400000) {
        *symbol = 206;
        return 10;
    } else {
        *symbol = 205;
        return 10;
    }

node_111100111:
    if (bits & 0x400000) {
        *symbol = 208;
        return 10;
    } else {
        *symbol = 207;
        return 10;
    }

node_111101:
    if (bits & 0x2000000) {
        goto node_1111011;
    } else {
        goto node_1111010;
    }

node_1111010:
    if (bits & 0x1000000) {
        goto node_11110101;
    } else {
        goto node_11110100;
    }

node_11110100:
    if (bits & 0x800000) {
        goto node_111101001;
    } else {
        goto node_111101000;
    }

node_111101000:
    if (bits & 0x400000) {
        *symbol = 210;
        return 10;
    } else {
        *symbol = 209;
        return 10;
    }

node_111101001:
    if (bits & 0x400000) {
        *symbol = 212;
        return 10;
    } else {
        *symbol = 211;
        return 10;
    }

node_11110101:
    if (bits & 0x800000) {
        goto node_111101011;
    } else {
        goto node_111101010;
    }

node_111101010:
    if (bits & 0x400000) {
        *symbol = 214;
        return 10;
    } else {
        *symbol = 213;
        return 10;
    }

node_111101011:
    if (bits & 0x400000) {
        *symbol = 216;
        return 10;
    } else {
        *symbol = 215;
        return 10;
    }

node_1111011:
    if (bits & 0x1000000) {
        goto node_11110111;
    } else {
        goto node_11110110;
    }

node_11110110:
    if (bits & 0x800000) {
        goto node_111101101;
    } else {
        goto node_111101100;
    }

node_111101100:
    if (bits & 0x400000) {
        *symbol = 218;
        return 10;
    } else {
        *symbol = 217;
        return 10;
    }

node_111101101:
    if (bits & 0x400000) {
        *symbol = 220;
        return 10;
    } else {
        *symbol = 219;
        return 10;
    }

node_11110111:
    if (bits & 0x800000) {
        goto node_111101111;
    } else {
        goto node_111101110;
    }

node_111101110:
    if (bits & 0x400000) {
        *symbol = 222;
        return 10;
    } else {
        *symbol = 221;
        return 10;
    }

node_111101111:
    if (bits & 0x400000) {
        *symbol = 224;
        return 10;
    } else {
        *symbol = 223;
        return 10;
    }

node_11111:
    if (bits & 0x4000000) {
        goto node_111111;
    } else {
        goto node_111110;
    }

node_111110:
    if (bits & 0x2000000) {
        goto node_1111101;
    } else {
        goto node_1111100;
    }

node_1111100:
    if (bits & 0x1000000) {
        goto node_11111001;
    } else {
        goto node_11111000;
    }

node_11111000:
    if (bits & 0x800000) {
        goto node_111110001;
    } else {
        goto node_111110000;
    }

node_111110000:
    if (bits & 0x400000) {
        *symbol = 226;
        return 10;
    } else {
        *symbol = 225;
        return 10;
    }

node_111110001:
    if (bits & 0x400000) {
        *symbol = 228;
        return 10;
    } else {
        *symbol = 227;
        return 10;
    }

node_11111001:
    if (bits & 0x800000) {
        goto node_111110011;
    } else {
        goto node_111110010;
    }

node_111110010:
    if (bits & 0x400000) {
        *symbol = 230;
        return 10;
    } else {
        *symbol = 229;
        return 10;
    }

node_111110011:
    if (bits & 0x400000) {
        *symbol = 232;
        return 10;
    } else {
        *symbol = 231;
        return 10;
    }

node_1111101:
    if (bits & 0x1000000) {
        goto node_11111011;
    } else {
        goto node_11111010;
    }

node_11111010:
    if (bits & 0x800000) {
        goto node_111110101;
    } else {
        goto node_111110100;
    }

node_111110100:
    if (bits & 0x400000) {
        *symbol = 234;
        return 10;
    } else {
        *symbol = 233;
        return 10;
    }

node_111110101:
    if (bits & 0x400000) {
        *symbol = 236;
        return 10;
    } else {
        *symbol = 235;
        return 10;
    }

node_11111011:
    if (bits & 0x800000) {
        goto node_111110111;
    } else {
        goto node_111110110;
    }

node_111110110:
    if (bits & 0x400000) {
        *symbol = 238;
        return 10;
    } else {
        *symbol = 237;
        return 10;
    }

node_111110111:
    if (bits & 0x400000) {
        *symbol = 240;
        return 10;
    } else {
        *symbol = 239;
        return 10;
    }

node_111111:
    if (bits & 0x2000000) {
        goto node_1111111;
    } else {
        goto node_1111110;
    }

node_1111110:
    if (bits & 0x1000000) {
        goto node_11111101;
    } else {
        goto node_11111100;
    }

node_11111100:
    if (bits & 0x800000) {
        goto node_111111001;
    } else {
        goto node_111111000;
    }

node_111111000:
    if (bits & 0x400000) {
        *symbol = 242;
        return 10;
    } else {
        *symbol = 241;
        return 10;
    }

node_111111001:
    if (bits & 0x400000) {
        *symbol = 244;
        return 10;
    } else {
        *symbol = 243;
        return 10;
    }

node_11111101:
    if (bits & 0x800000) {
        goto node_111111011;
    } else {
        goto node_111111010;
    }

node_111111010:
    if (bits & 0x400000) {
        *symbol = 246;
        return 10;
    } else {
        *symbol = 245;
        return 10;
    }

node_111111011:
    if (bits & 0x400000) {
        *symbol = 248;
        return 10;
    } else {
        *symbol = 247;
        return 10;
    }

node_1111111:
    if (bits & 0x1000000) {
        goto node_11111111;
    } else {
        goto node_11111110;
    }

node_11111110:
    if (bits & 0x800000) {
        goto node_111111101;
    } else {
        goto node_111111100;
    }

node_111111100:
    if (bits & 0x400000) {
        *symbol = 250;
        return 10;
    } else {
        *symbol = 249;
        return 10;
    }

node_111111101:
    if (bits & 0x400000) {
        *symbol = 252;
        return 10;
    } else {
        *symbol = 251;
        return 10;
    }

node_11111111:
    if (bits & 0x800000) {
        goto node_111111111;
    } else {
        goto node_111111110;
    }

node_111111110:
    if (bits & 0x400000) {
        *symbol = 254;
        return 10;
    } else {
        *symbol = 253;
        return 10;
    }

node_111111111:
    if (bits & 0x400000) {
        return 0; /* invalid node */
    } else {
        *symbol = 255;
        return 10;
    }
}

struct aws_huffman_symbol_coder *test_get_coder(void) {

    static struct aws_huffman_symbol_coder coder = {
        .encode = encode_symbol,
        .decode = decode_symbol,
        .userdata = NULL,
    };
    return &coder;
}
