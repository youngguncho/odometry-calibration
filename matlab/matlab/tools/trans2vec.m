function v=trans2vec(A)
  v = [A(1:2,3); atan2(A(2,1),A(1,1))];
end
