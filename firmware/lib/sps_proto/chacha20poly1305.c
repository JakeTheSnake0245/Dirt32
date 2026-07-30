/* RFC 8439 ChaCha20-Poly1305 AEAD — portable C99 implementation. */
#include "chacha20poly1305.h"
#include <string.h>

/* ---------------- ChaCha20 ---------------- */

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void st32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t rotl(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

#define QR(a, b, c, d)                          \
    a += b; d ^= a; d = rotl(d, 16);            \
    c += d; b ^= c; b = rotl(b, 12);            \
    a += b; d ^= a; d = rotl(d, 8);             \
    c += d; b ^= c; b = rotl(b, 7);

static void chacha20_block(const uint8_t key[32], uint32_t counter,
                           const uint8_t nonce[12], uint8_t out[64]) {
    uint32_t s[16], x[16];
    s[0] = 0x61707865; s[1] = 0x3320646e; s[2] = 0x79622d32; s[3] = 0x6b206574;
    for (int i = 0; i < 8; i++) s[4 + i] = le32(key + 4 * i);
    s[12] = counter;
    for (int i = 0; i < 3; i++) s[13 + i] = le32(nonce + 4 * i);

    memcpy(x, s, sizeof(x));
    for (int i = 0; i < 10; i++) {
        QR(x[0], x[4], x[8],  x[12]) QR(x[1], x[5], x[9],  x[13])
        QR(x[2], x[6], x[10], x[14]) QR(x[3], x[7], x[11], x[15])
        QR(x[0], x[5], x[10], x[15]) QR(x[1], x[6], x[11], x[12])
        QR(x[2], x[7], x[8],  x[13]) QR(x[3], x[4], x[9],  x[14])
    }
    for (int i = 0; i < 16; i++) st32(out + 4 * i, x[i] + s[i]);
}

static void chacha20_xor(const uint8_t key[32], uint32_t counter,
                         const uint8_t nonce[12],
                         const uint8_t *in, size_t len, uint8_t *out) {
    uint8_t block[64];
    while (len > 0) {
        chacha20_block(key, counter++, nonce, block);
        size_t n = len < 64 ? len : 64;
        for (size_t i = 0; i < n; i++) out[i] = in[i] ^ block[i];
        in += n; out += n; len -= n;
    }
}

/* ---------------- Poly1305 ---------------- */

typedef struct {
    uint32_t r[5], h[5], pad[4];
    size_t leftover;
    uint8_t buffer[16];
    uint8_t final;
} poly1305_ctx;

static void poly1305_init(poly1305_ctx *st, const uint8_t key[32]) {
    st->r[0] = (le32(key +  0)     ) & 0x3ffffff;
    st->r[1] = (le32(key +  3) >> 2) & 0x3ffff03;
    st->r[2] = (le32(key +  6) >> 4) & 0x3ffc0ff;
    st->r[3] = (le32(key +  9) >> 6) & 0x3f03fff;
    st->r[4] = (le32(key + 12) >> 8) & 0x00fffff;
    st->h[0] = st->h[1] = st->h[2] = st->h[3] = st->h[4] = 0;
    st->pad[0] = le32(key + 16); st->pad[1] = le32(key + 20);
    st->pad[2] = le32(key + 24); st->pad[3] = le32(key + 28);
    st->leftover = 0;
    st->final = 0;
}

static void poly1305_blocks(poly1305_ctx *st, const uint8_t *m, size_t bytes) {
    const uint32_t hibit = st->final ? 0 : (1u << 24);
    uint32_t r0 = st->r[0], r1 = st->r[1], r2 = st->r[2], r3 = st->r[3], r4 = st->r[4];
    uint32_t s1 = r1 * 5, s2 = r2 * 5, s3 = r3 * 5, s4 = r4 * 5;
    uint32_t h0 = st->h[0], h1 = st->h[1], h2 = st->h[2], h3 = st->h[3], h4 = st->h[4];

    while (bytes >= 16) {
        uint64_t d0, d1, d2, d3, d4;
        uint32_t c;
        h0 += (le32(m +  0)     ) & 0x3ffffff;
        h1 += (le32(m +  3) >> 2) & 0x3ffffff;
        h2 += (le32(m +  6) >> 4) & 0x3ffffff;
        h3 += (le32(m +  9) >> 6) & 0x3ffffff;
        h4 += (le32(m + 12) >> 8) | hibit;

        d0 = (uint64_t)h0 * r0 + (uint64_t)h1 * s4 + (uint64_t)h2 * s3 + (uint64_t)h3 * s2 + (uint64_t)h4 * s1;
        d1 = (uint64_t)h0 * r1 + (uint64_t)h1 * r0 + (uint64_t)h2 * s4 + (uint64_t)h3 * s3 + (uint64_t)h4 * s2;
        d2 = (uint64_t)h0 * r2 + (uint64_t)h1 * r1 + (uint64_t)h2 * r0 + (uint64_t)h3 * s4 + (uint64_t)h4 * s3;
        d3 = (uint64_t)h0 * r3 + (uint64_t)h1 * r2 + (uint64_t)h2 * r1 + (uint64_t)h3 * r0 + (uint64_t)h4 * s4;
        d4 = (uint64_t)h0 * r4 + (uint64_t)h1 * r3 + (uint64_t)h2 * r2 + (uint64_t)h3 * r1 + (uint64_t)h4 * r0;

        c = (uint32_t)(d0 >> 26); h0 = (uint32_t)d0 & 0x3ffffff;
        d1 += c; c = (uint32_t)(d1 >> 26); h1 = (uint32_t)d1 & 0x3ffffff;
        d2 += c; c = (uint32_t)(d2 >> 26); h2 = (uint32_t)d2 & 0x3ffffff;
        d3 += c; c = (uint32_t)(d3 >> 26); h3 = (uint32_t)d3 & 0x3ffffff;
        d4 += c; c = (uint32_t)(d4 >> 26); h4 = (uint32_t)d4 & 0x3ffffff;
        h0 += c * 5; c = h0 >> 26; h0 &= 0x3ffffff;
        h1 += c;

        m += 16; bytes -= 16;
    }
    st->h[0] = h0; st->h[1] = h1; st->h[2] = h2; st->h[3] = h3; st->h[4] = h4;
}

static void poly1305_update(poly1305_ctx *st, const uint8_t *m, size_t bytes) {
    if (st->leftover) {
        size_t want = 16 - st->leftover;
        if (want > bytes) want = bytes;
        memcpy(st->buffer + st->leftover, m, want);
        bytes -= want; m += want; st->leftover += want;
        if (st->leftover < 16) return;
        poly1305_blocks(st, st->buffer, 16);
        st->leftover = 0;
    }
    if (bytes >= 16) {
        size_t want = bytes & ~(size_t)15;
        poly1305_blocks(st, m, want);
        m += want; bytes -= want;
    }
    if (bytes) {
        memcpy(st->buffer, m, bytes);
        st->leftover = bytes;
    }
}

static void poly1305_finish(poly1305_ctx *st, uint8_t mac[16]) {
    uint32_t h0, h1, h2, h3, h4, c, g0, g1, g2, g3, g4, mask;
    uint64_t f;

    if (st->leftover) {
        size_t i = st->leftover;
        st->buffer[i++] = 1;
        for (; i < 16; i++) st->buffer[i] = 0;
        st->final = 1;
        poly1305_blocks(st, st->buffer, 16);
    }

    h0 = st->h[0]; h1 = st->h[1]; h2 = st->h[2]; h3 = st->h[3]; h4 = st->h[4];
    c = h1 >> 26; h1 &= 0x3ffffff;
    h2 += c; c = h2 >> 26; h2 &= 0x3ffffff;
    h3 += c; c = h3 >> 26; h3 &= 0x3ffffff;
    h4 += c; c = h4 >> 26; h4 &= 0x3ffffff;
    h0 += c * 5; c = h0 >> 26; h0 &= 0x3ffffff;
    h1 += c;

    g0 = h0 + 5; c = g0 >> 26; g0 &= 0x3ffffff;
    g1 = h1 + c; c = g1 >> 26; g1 &= 0x3ffffff;
    g2 = h2 + c; c = g2 >> 26; g2 &= 0x3ffffff;
    g3 = h3 + c; c = g3 >> 26; g3 &= 0x3ffffff;
    g4 = h4 + c - (1u << 26);

    mask = (g4 >> 31) - 1;
    g0 &= mask; g1 &= mask; g2 &= mask; g3 &= mask; g4 &= mask;
    mask = ~mask;
    h0 = (h0 & mask) | g0; h1 = (h1 & mask) | g1; h2 = (h2 & mask) | g2;
    h3 = (h3 & mask) | g3; h4 = (h4 & mask) | g4;

    h0 = (h0 | (h1 << 26)) & 0xffffffff;
    h1 = ((h1 >> 6) | (h2 << 20)) & 0xffffffff;
    h2 = ((h2 >> 12) | (h3 << 14)) & 0xffffffff;
    h3 = ((h3 >> 18) | (h4 << 8)) & 0xffffffff;

    f = (uint64_t)h0 + st->pad[0];              h0 = (uint32_t)f;
    f = (uint64_t)h1 + st->pad[1] + (f >> 32);  h1 = (uint32_t)f;
    f = (uint64_t)h2 + st->pad[2] + (f >> 32);  h2 = (uint32_t)f;
    f = (uint64_t)h3 + st->pad[3] + (f >> 32);  h3 = (uint32_t)f;

    st32(mac + 0, h0); st32(mac + 4, h1); st32(mac + 8, h2); st32(mac + 12, h3);
    memset(st, 0, sizeof(*st));
}

/* ---------------- AEAD construction (RFC 8439 §2.8) ---------------- */

static void aead_mac(const uint8_t key[32], const uint8_t nonce[12],
                     const uint8_t *aad, size_t aad_len,
                     const uint8_t *ct, size_t ct_len, uint8_t tag[16]) {
    uint8_t polykey[64];
    static const uint8_t zeros[16] = {0};
    uint8_t lens[16];
    poly1305_ctx st;

    chacha20_block(key, 0, nonce, polykey); /* first 32 bytes = poly key */
    poly1305_init(&st, polykey);
    memset(polykey, 0, sizeof(polykey));

    poly1305_update(&st, aad, aad_len);
    if (aad_len % 16) poly1305_update(&st, zeros, 16 - (aad_len % 16));
    poly1305_update(&st, ct, ct_len);
    if (ct_len % 16) poly1305_update(&st, zeros, 16 - (ct_len % 16));

    for (int i = 0; i < 8; i++) lens[i]     = (uint8_t)((uint64_t)aad_len >> (8 * i));
    for (int i = 0; i < 8; i++) lens[8 + i] = (uint8_t)((uint64_t)ct_len  >> (8 * i));
    poly1305_update(&st, lens, 16);
    poly1305_finish(&st, tag);
}

void cc20p1305_encrypt(const uint8_t key[32], const uint8_t nonce[12],
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *pt, size_t pt_len,
                       uint8_t *ct, uint8_t tag[16]) {
    chacha20_xor(key, 1, nonce, pt, pt_len, ct);
    aead_mac(key, nonce, aad, aad_len, ct, pt_len, tag);
}

int cc20p1305_decrypt(const uint8_t key[32], const uint8_t nonce[12],
                      const uint8_t *aad, size_t aad_len,
                      const uint8_t *ct, size_t ct_len,
                      const uint8_t tag[16], uint8_t *pt) {
    uint8_t calc[16];
    uint8_t diff = 0;
    aead_mac(key, nonce, aad, aad_len, ct, ct_len, calc);
    for (int i = 0; i < 16; i++) diff |= (uint8_t)(calc[i] ^ tag[i]);
    memset(calc, 0, sizeof(calc));
    if (diff != 0) return -1;
    chacha20_xor(key, 1, nonce, ct, ct_len, pt);
    return 0;
}
