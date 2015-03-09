function C=segment(A,t)
    B = zeros(size(A));
    B = B(:,:,1:2);
    B(:,:,1) = A(:,:,1)-A(:,:,2);
    B(:,:,2) = A(:,:,1)-A(:,:,3);
    C = min(B(:,:,1),B(:,:,2))
    C(C<t) = 0;
    C(C>=t) = 1;
end