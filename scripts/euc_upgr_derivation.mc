/*
This is a derivation of the coefficients of the A and B matrix, as described
in J. Oliensis's paper on linear self-calibration. This is for the case of
known calibration, where one is upgrading an existing projective reconstruction
to metric with known calibration.
The script maxima.py converts the output into C and Python code.
Keir Mierle 2007
*/

tr(a) := block([ans:0],for i thru length(a) do ans:ans+a[i,i],ans);

/* For some reason easy gradient fuctionality is not built right into yacas */
grad(expr, vs) := block(
    [g:[]],
    for i thru length(vs) do
        g:cons(diff(expr, vs[i], 1),g),
    reverse(g)
);

q : [q0,q1,q2,q3,q4,q5,q6,q7,q8,q9];

Q : matrix([q0,q1,q2,q3],
           [q1,q4,q5,q6],
           [q2,q5,q7,q8],
           [q3,q6,q8,q9]);

P : matrix([p0,p1,p2, p3],
           [p4,p5,p6, p7],
           [p8,p9,p10,p11]);

w : matrix([w1,w2,w3],
           [w2,w4,w5],
           [w3,w5,w6]);

qTq : matrix(outermap(lambda ([x,y],x*y), q,q));

/* Note that beforehand trww needs to be defined. */
X : P . Q . transpose(P);
E_euc : tr(X . X) - ((tr(w . X))^2) / trww;

Aq : grad(E_euc, q);

Aqz : [];
for i thru length(Aq) do
	Aqz : cons(Aq[i]=0, Aqz);
Aqz : reverse(Aqz);

A : coefmatrix(Aqz, q);
stringout(A_matrix, A);

Bq : grad(tr(Q.Q), q);
B : coefmatrix(Bq, q);
stringout(B_matrix, B);

/*
 * Then use python or something else to parse the outputted matrix literal into
 * C or python code. WARNING: the code above seems to stress Maxima out really
 * good; it took a full 3 minutes to finish the above coefmatrix extraction on 
 * my machine. 
 */
