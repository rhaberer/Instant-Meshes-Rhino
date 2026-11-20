// Modified serial C# adaptation of ext/dset/dset.h. See LICENSE-dset.
namespace InstantMeshes.Core;

internal sealed class DisjointSets
{
    private readonly int[] parents;
    private readonly byte[] ranks;

    public DisjointSets(int count)
    {
        parents = new int[count];
        ranks = new byte[count];
        for (int i = 0; i < count; i++)
        {
            parents[i] = i;
        }
    }

    public int Find(int vertex)
    {
        while (parents[vertex] != vertex)
        {
            parents[vertex] = parents[parents[vertex]];
            vertex = parents[vertex];
        }
        return vertex;
    }

    public int Unite(int a, int b)
    {
        a = Find(a);
        b = Find(b);
        if (a == b)
        {
            return a;
        }
        // Equal-rank roots use the lower index to keep later graph ordering deterministic.
        if (ranks[a] < ranks[b] || (ranks[a] == ranks[b] && a > b))
        {
            (a, b) = (b, a);
        }

        if (ranks[a] == ranks[b])
        {
            ranks[a]++;
        }

        parents[b] = a;
        return a;
    }
}
