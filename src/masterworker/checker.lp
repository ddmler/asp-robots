bug(ID1, T) :- robpos(ID1,N,M,T),robpos(ID2,N,M,T), ID1<ID2.

#show bug/2.
