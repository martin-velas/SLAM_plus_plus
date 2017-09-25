This is a small script which prints information about a block matrix layout file in the .bla
format. Use "blanalyze.sh matrix.bla" to get this sort of output:

> 1596138 x 1596138 elements (55901412 nnz, 0.00219423% dense)
> 531175 x 531175 blocks (3369915 nnz, 0.00119438% dense)
> VBR would take 490.072 MB (double precision, 64-bit indices)
> block-row-count[3] = 530304
> block-row-count[6] = 871
> block-col-count[3] = 530304
> block-col-count[6] = 871

The first line is the size of the matrix in elements and the number of nonzeros and sparsity.

The second line is the size of the matrix in blocks, followed again by the number of nonzeros
and pseudo-sparsity in blocks.

The third line shows the size in memory which such a matrix would take if represented using
the variable block row (VBR) format.

Finally, the last few lines contain the histogram of block row and block column sizes. This
example shows a matrix with block rows and block columns of sizes 3 and 6 elements. There
are 871 block rows and block columns of size 6 and 530304 of size 3 (the matrix in question
is a symmetric one). The order in which the sizes go is arbitrary.
