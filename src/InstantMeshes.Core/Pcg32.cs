// Modified C# adaptation of Wenzel Jakob's pcg32.h, February 2015.
// PCG random number generator developed by Melissa O'Neill.
// Licensed under Apache-2.0. See LICENSE-pcg32.
using System.Numerics;

namespace InstantMeshes.Core;

internal struct Pcg32()
{
    private ulong state = 0x853c49e6748fea9b;

    public uint NextUInt()
    {
        ulong previous = state;
        state = unchecked(previous * 0x5851f42d4c957f2dUL + 0xda3e39cb94b95bdbUL);
        return BitOperations.RotateRight((uint)(((previous >> 18) ^ previous) >> 27), (int)(previous >> 59));
    }

    public int NextIndex(int length)
    {
        uint bound = (uint)length;
        uint threshold = unchecked(0u - bound) % bound;
        uint value;
        do
        {
            value = NextUInt();
        }
        while (value < threshold);
        return (int)(value % bound);
    }

    public float NextFloat() =>
        BitConverter.UInt32BitsToSingle(0x3f800000u | (NextUInt() >> 9)) - 1f;
}
