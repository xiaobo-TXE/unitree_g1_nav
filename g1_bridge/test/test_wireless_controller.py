from g1_bridge.wireless_controller import decode_buttons


def test_decode_buttons_empty():
    assert decode_buttons(0) == ()


def test_decode_buttons_uses_unitree_bit_layout():
    keys = (1 << 0) | (1 << 11) | (1 << 15)
    assert decode_buttons(keys) == ('R1', 'Y', 'LEFT')


def test_decode_buttons_is_limited_to_uint16():
    assert decode_buttons(1 << 16) == ()
