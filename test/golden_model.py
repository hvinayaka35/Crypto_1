"""Golden model for the crypto-agile NTT butterfly.

SCHEMES[sel] -> (name, q, k).  k is the number of Montgomery loop iterations,
chosen as ceil(log2(q)) so that every residue fits in k bits.
"""

SCHEMES = {
    0: ("ML-KEM", 3329, 12),
    1: ("FN-DSA", 12289, 14),
    2: ("ML-DSA", 8380417, 23),
    3: ("ML-DSA", 8380417, 23),
}


def to_mont(x, q, k):
    """Map x into the Montgomery domain: x * 2^k mod q."""
    return (x * pow(2, k, q)) % q


def mont_mul(A, B, q, k):
    """Bit-serial Montgomery product, cycle-for-cycle identical to the RTL.

    Returns A*B*2^-k mod q.  The loop invariant S < 2q is asserted every
    iteration, and the pre-shift sum is checked to be even (which is what
    the m bit guarantees).
    """
    S = 0
    for i in range(k):
        ai = (A >> i) & 1
        m = (S ^ (ai & (B & 1))) & 1
        s = S + (B if ai else 0) + (q if m else 0)
        assert s % 2 == 0, "m bit failed to clear the LSB"
        S = s >> 1
        assert S < 2 * q, "Montgomery invariant S < 2q violated"
    return S - q if S >= q else S


def butterfly(a, b, w, sel):
    """Cooley-Tukey butterfly.  w is passed in the *normal* domain here; the
    model converts it, mirroring what the host must do before loading."""
    _, q, k = SCHEMES[sel]
    t = mont_mul(b, to_mont(w, q, k), q, k)
    return (a + t) % q, (a - t) % q, t


def cycles(sel):
    """Latency of one butterfly in clock cycles, excluding I/O shifting."""
    _, _, k = SCHEMES[sel]
    return k + 5
