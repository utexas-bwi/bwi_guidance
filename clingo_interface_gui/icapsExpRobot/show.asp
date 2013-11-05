inside(P,R,V_step) :- h(eql(inside(P,R),true),V_step).
n_inside(P,R,V_step) :- h(eql(inside(P,R),false),V_step).
beside(D,V_step) :- h(eql(beside(D),true),V_step).
n_beside(D,V_step) :- h(eql(beside(D),false),V_step).
facing(D,V_step) :- h(eql(facing(D),true),V_step).
n_facing(D,V_step) :- h(eql(facing(D),false),V_step).
believeinside(P,R,V_step) :- h(eql(believeinside(P,R),true),V_step).
n_believeinside(P,R,V_step) :- h(eql(believeinside(P,R),false),V_step).
at(R,V_step) :- h(eql(loc,R),V_step).
open(D,V_step) :- h(eql(open(D),true),V_step).
n_open(D,V_step) :- h(eql(open(D),false),V_step).
visited(P,V_step) :- h(eql(visited(P),true),V_step).
n_visited(P,V_step) :- h(eql(visited(P),false),V_step).
served(P,V_step) :- h(eql(served(P),true),V_step).
n_served(P,V_step) :- h(eql(served(P),false),V_step).
acc(R,D,R2) :- h(eql(acc(R,D,R2),true)).
hasdoor(R,D) :- h(eql(hasdoor(R,D),true)).

approach(D,V_step) :- occ(eql(approach(D),true),V_step).
gothrough(D,V_step) :- occ(eql(gothrough(D),true),V_step).
callforopen(D,V_step) :- occ(eql(callforopen(D),true),V_step).
goto(P,V_step) :- occ(eql(goto(P),true),V_step).
sense(P1,ploc(P2),V_step) :- occ(eql(sense(P1,ploc(P2)),true),V_step).

h(eql(beside(D),true),V_step) :- beside(D,V_step).
h(eql(beside(D),false),V_step) :- n_beside(D,V_step).
h(eql(facing(D),true),V_step) :- facing(D,V_step).
h(eql(facing(D),false),V_step) :- n_facing(D,V_step).
h(eql(loc,R),V_step) :- at(R,V_step).
h(eql(open(D),true),V_step) :- open(D,V_step).
h(eql(open(D),false),V_step) :- n_open(D,V_step).

cost(@dis(D1,D2,R),V_step) :- approach(D1,V_step), beside(D2,V_step), at(R,V_step).
cost(10,V_step) :- approach(D,V_step), {beside(Y,V_step)}0.
cost(1,V_step) :- sense(P1,ploc(P),V_step).
cost(5,V_step) :- gothrough(D,V_step).
cost(1,V_step) :- goto(P,V_step).
cost(1,V_step) :- callforopen(D,V_step).

#minimize[cost(X,Y)=X@2].

#hide.
#show approach/2.
#show gothrough/2.
#show callforopen/2.
#show goto/2.
#show sense/3.

#show inside/3.
#show n_inside/3.
#show believeinside/3.
#show n_believeinside/3.

#show beside/2.
#show n_beside/2.
#show facing/2.
#show n_facing/2.
#show at/2.
#show open/2.
#show n_open/2.
#show visited/2.
#show n_visited/2.
#show served/2.
#show n_served/2.

%#show v_approach/2.
%#show v_gothrough/2.
%#show v_callforopen/2.
%#show noop/1.

