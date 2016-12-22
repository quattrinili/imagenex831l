#!/usr/bin/env python

"""
    Author: Alberto Quattrini Li
    Affiliation: AFRL - University of South Carolina
    Date: 12/22/2016

    Description:
    Binary representation utilities.

  
    TODO Make a library.
"""

def twos_complement(input_value, num_bits):
	"""Calculates a two's complement integer from the given input value's bits.
    
    Source: https://en.wikipedia.org/wiki/Two's_complement#Converting_from_two.27s_complement_representation

    Args:
        input_value(unsigned int): unsigned int corresponding to binary 
            representation.
        num_bits(unsigned int): number of bits for two-complement's integer.
    Returns:
        Two's complement value (int).
    """
	mask = 2 ** (num_bits - 1)
	return -(input_value & mask) + (input_value & ~mask)

def convert_to_weighted_bits(bits, weight, num_bits=14, 
    least_significant_bit_zero=False):
    """Two complement's value to number according to weight.

    0th bit * 2^0 * weight + 1st bit * 2^1 * weight + ...

    Args:
        bits(string): binary representation of the number got by bin function.
            Note that bin function on a negative number returns a string
            that has '-' at the beginning and returns a binary representation
            of the unsigned value.
        weight: number to change weight of each bit.
        num_bits(unsigned int): number of bits for two-complement's integer.
        least_significant_bit_zero(bool): zero index corresponds to LSB (True)
            or MSB (False).
    Returns:
        Two's complement value changed according to weight.
    """
    if bits[0] == '-':
        sign = -1
        offset = 3
    else:
        sign = 1
        offset = 2

    if least_significant_bit_zero:
        # 0: least significant bit.
        bits = bits[offset:]
    else:
        bits = bits[offset:][::-1]

    result = 0
    for i in range(0, len(bits)):
        result += int(bits[i]) * (2 ** i) * weight

    result = result * sign
    return result

def my_round(x, base=5):
    """Rounding a number to base.
    """
    return int(base * round(float(x)/base))
