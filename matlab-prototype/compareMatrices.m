function diff = compareMatrices(pathToMatrix1, pathToMatrix2)

    M1 = readmatrix(pathToMatrix1);
    M2 = readmatrix(pathToMatrix2);
    
    assert(size(M1,1) == size(M2,1), 'Matrices have different number of rows!');
    assert(size(M1,2) == size(M2,2), 'Matrices have different number of columns!');
    
    diff = M1(:) - M2(:);
    
    bar(diff);
    grid on;
    ylabel('diff');

end