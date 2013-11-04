% Standard description-independent declarations and laws to CCalc-ify ASP

% A derived binary relation between a constant and its domain objects,
% connected via constant_sort and sort_object

%-------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------------------------------------------

query_label(query) :- true.
% Selects which query to activate in a based on the value of the const "query"

constant_object(V_constant,Object) :- constant_sort(V_constant,V_sort),sort_object(V_sort,Object).

% constant_posobject relates constants with their positive objects
% positive objects are defined to be any object that is not 'none'
% (multi-valued) or 'false' (boolean)

constant_posobject(V_constant,Object) :- constant_object(V_constant,Object),constant_sort(V_constant,boolean),Object!=false.

constant_posobject(V_constant,Object) :- constant_object(V_constant,Object),not constant_sort(V_constant,boolean),Object!=none.

% constant_negobject relates constants with their negative object
% negative objects are false (boolean) and none (non-boolean)
constant_negobject(V_constant,false) :- constant_sort(V_constant,boolean).

constant_negobject(V_constant,none) :- not constant_sort(V_constant,boolean),constant(V_constant).


%-------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------------------------------------------

% Standard, description-independent declarations of sorts and domains

step(0..maxstep).
astep(1..maxstep).

#domain step(V_step).
#domain astep(V_astep).

sort(boolean) :- true.
#domain boolean(V_boolean).
sort_object(boolean,V_boolean) :- true.

boolean(true) :- true.
boolean(false) :- true.


sort(computed) :- true.
#domain computed(V_computed).
sort_object(computed,V_computed) :- true.

% Constants hierarchy

% Meta-constants to group categories of constants

sort(constant) :- true.
#domain constant(V_constant).
sort_object(constant,V_constant) :- true.

%----

% Rigid constants
sort(rigid) :- true.
#domain rigid(V_rigid).
sort_object(rigid,V_rigid) :- true.

%---

% Fluent-based constants

sort(fluent) :- true.
#domain fluent(V_fluent).
sort_object(fluent,V_fluent) :- true.

sort(simpleFluent) :- true.
#domain simpleFluent(V_simpleFluent).
sort_object(simpleFluent,V_simpleFluent) :- true.

sort(inertialFluent) :- true.
#domain inertialFluent(V_inertialFluent).
sort_object(inertialFluent,V_inertialFluent) :- true.

sort(sdFluent) :- true.
#domain sdFluent(V_sdFluent).
sort_object(sdFluent,V_sdFluent) :- true.

sort(abFluent) :- true.
#domain abFluent(V_abFluent).
sort_object(abFluent,V_abFluent) :- true.

%---

% Action-based constants

sort(action) :- true.
#domain action(V_action).
sort_object(action,V_action) :- true.

sort(exogenousAction) :- true.
#domain exogenousAction(V_exogenousAction).
sort_object(exogenousAction,V_exogenousAction) :- true.

sort(abAction) :- true.
#domain abAction(V_abAction).
sort_object(abAction,V_abAction) :- true.

sort(attribute) :- true.
#domain attribute(V_attribute).
sort_object(attribute,V_attribute) :- true.

% A regular action is defined to be any action which isn't an abAction or attribute
sort(regularAction) :- true.
#domain regularAction(V_regularAction).
sort_object(regularAction,V_regularAction) :- true.

%---

% Abnormality constants

sort(abnormality) :- true.
#domain abnormality(V_abnormality).
sort_object(abnormality,V_abnormality) :- true.

sort(staticAbnormality) :- true.
#domain staticAbnormality(V_staticAbnormality).
sort_object(staticAbnormality,V_staticAbnormality) :- true.

sort(dynamicAbnormality) :- true.
#domain dynamicAbnormality(V_dynamicAbnormality).
sort_object(dynamicAbnormality,V_dynamicAbnormality) :- true.

%---

% Subsort relations

constant(V_fluent) :- true.
constant(V_action) :- true.
constant(V_rigid) :- true.
constant(V_abnormality) :- true.
fluent(V_simpleFluent) :- true.
simpleFluent(V_inertialFluent) :- true.
fluent(V_sdFluent) :- true.
sdFluent(V_abFluent) :- true.
action(V_exogenousAction) :- true.
action(V_abAction) :- true.
action(V_attribute) :- true.

regularAction(V_action) :- not attribute(V_action),not abAction(V_action).

abnormality(V_staticAbnormality) :- true.
abnormality(V_dynamicAbnormality) :- true.

% Note: This isn't necessary for abnormality constants


%-------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------------------------------------------

%%% Description-independent rules to CCalc-ify ASP @ t=0

% Exogeneity for exogenous actions
exogenous(V_exogenousAction) :- true.

% Exogeneity for attributes
exogenous(V_attribute) :- true.

% Inertia for inertial fluents
inertial(V_inertialFluent) :- true.

% anyInitialState: Exogeneity for simple fluents at time 0
{h(eql(V_simpleFluent,Object),0)} :- constant_object(V_simpleFluent,Object).


% exogenous: Grants exogeneity to a constant

% Rigids
{h(eql(V_rigid,Value))} :- exogenous(V_rigid),constant_object(V_rigid,Value).

% Existence and uniqueness for every constant relative to its domain

% Rigid
false :- not 1{h(eql(V_rigid,Object1)):constant_object(V_rigid,Object1)}1.


%-------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------------------------------------------

%%% Description-independent rules to CCalc-ify ASP @ t>0

% abActions default to false
{occ(eql(V_abAction,false),V_astep-1)} :- true.

% Default false for abFluents
{h(eql(V_abFluent,X),V_step)} :- constant_negobject(V_abFluent,X).


% Restriction that attributes must take on the value "none" if and only if their linked action does not execute
false :- {not occ(eql(V_attribute,NegObj1),V_astep-1)}0,constant_negobject(V_action,NegObj2),action_attribute(V_action,V_attribute),constant_negobject(V_attribute,NegObj1),not occ(eql(V_action,NegObj2),V_astep-1).
false :- {not occ(eql(V_action,NegObj2),V_astep-1)}0,constant_negobject(V_action,NegObj2),action_attribute(V_action,V_attribute),constant_negobject(V_attribute,NegObj1),not occ(eql(V_attribute,NegObj1),V_astep-1).


% exogenous: Grants exogeneity to a constant

% Actions
{occ(eql(V_action,Value),V_astep-1)} :- exogenous(V_action),constant_object(V_action,Value).

% Fluents
{h(eql(V_fluent,Value),V_step)} :- exogenous(V_fluent),constant_object(V_fluent,Value).

% Abnormality constants should also be exogenous
{ab_h(V_staticAbnormality,V_step)} :- true.
{ab_h(V_dynamicAbnormality,V_astep-1)} :- true.

% inertial: Grants inertia to a (non-rigid) fluent

{h(eql(V_fluent,Value),V_astep)} :- inertial(V_fluent),h(eql(V_fluent,Value),V_astep-1),constant_object(V_fluent,Value).

% noconcurrency: If stated as a fact, prevents concurrent execution of Boolean actions

false :- noconcurrency,2{occ(eql(X_action,true),V_astep-1):action(X_action)}.

% strong_noconcurrency: A stronger version of noconcurrency which prevents 2 of any non-attribute, non abAction 
% action from occurring simultaneously

false :- strong_noconcurrency,2{occ(eql(X_action,X_Object),V_astep-1):regularAction(X_action):constant_posobject(X_action,X_Object)}.

% Existence and uniqueness for every constant relative to its domain

% Fluents (@ t>0)
false :- not 1{h(eql(V_fluent,Object1),V_step):constant_object(V_fluent,Object1)}1.

% Actions
false :- not 1{occ(eql(V_action,Object1),V_astep-1):constant_object(V_action,Object1)}1.


%-------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------------------------------------------

% Etc
#hide constant_object/2.
#hide constant_posobject/2.
#hide constant_negobject/2.
#hide constant_sort/2.
#hide sort_object/2.
#hide action_attribute/2.
#hide sort/1.
#hide sort_object/2.
#hide boolean/1.
#hide step/1.
#hide astep/1.
#hide constant/1.
#hide fluent/1.
#hide simpleFluent/1.
#hide inertialFluent/1.
#hide sdFluent/1.
#hide abFluent/1.
#hide action/1.
#hide exogenousAction/1.
#hide abAction/1.
#hide attribute/1.
#hide regularAction/1.
#hide abnormality/1.
#hide staticAbnormality/1.
#hide dynamicAbnormality/1.
#hide exogenous/1.
#hide inertial/1.
#hide noconcurrency.
#hide strong_noconcurrency.
#hide query_label/1.
#hide rigid/1.
#hide computed/1.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% File "/v/filer4b/v20q001/fkyang/navreal"
sort(office) :- true.
#domain office(V_office).
sort_object(office,V_office) :- true.
#hide office(V_office).

room(V_office) :- true.
sort(room) :- true.
#domain room(V_room).
sort_object(room,V_room) :- true.
#hide room(V_room).

sort(door) :- true.
#domain door(V_door).
sort_object(door,V_door) :- true.
#hide door(V_door).

sort(person) :- true.
#domain person(V_person).
sort_object(person,V_person) :- true.
#hide person(V_person).

person(alice) :- true.
person(bob) :- true.
person(carol) :- true.
person(dan) :- true.
person(erin) :- true.
person(frank) :- true.
person(george) :- true.
person(harry) :- true.
person(ian) :- true.
person(jack) :- true.

office(o3_508) :- true.
office(o3_510) :- true.
office(o3_512) :- true.
office(o3_416) :- true.
office(l3_414a) :- true.
office(s3_516) :- true.
office(l3_436) :- true.
office(o3_428) :- true.
office(o3_426) :- true.
office(l3_414b) :- true.
office(l3_414) :- true.
office(o3_402) :- true.
office(o3_412) :- true.
office(o3_502) :- true.
office(o3_404) :- true.
office(o3_418) :- true.
office(o3_420) :- true.
office(o3_422) :- true.
office(o3_430) :- true.
office(o3_432) :- true.

room(cor) :- true.

door(d3_508) :- true.
door(d3_510) :- true.
door(d3_512) :- true.
door(d3_416) :- true.
door(d3_414a1) :- true.
door(d3_414a2) :- true.
door(d3_414a3) :- true.
door(d3_5161) :- true.
door(d3_5162) :- true.
door(d3_4361) :- true.
door(d3_4362) :- true.
door(d3_428) :- true.
door(d3_426) :- true.
door(d3_414b1) :- true.
door(d3_414b2) :- true.
door(d3_414b3) :- true.
door(d3_402) :- true.
door(d3_412) :- true.
door(d3_502) :- true.
door(d3_404) :- true.
door(d3_418) :- true.
door(d3_420) :- true.
door(d3_422) :- true.
door(d3_430) :- true.
door(d3_432) :- true.

#domain door(D).
#domain door(D1).
#domain door(D2).
#domain door(D3).
#domain person(P).
#domain person(P1).
#domain person(P2).
#domain person(P3).
#domain room(R).
#domain room(R1).
#domain room(R2).
#domain room(R3).
#domain office(O).
#domain office(O1).
#domain office(O2).
#domain office(O3).

rigid(hasdoor(V_room,V_door)) :- true.
constant_sort(hasdoor(V_room,V_door),boolean) :- true.
rigid(acc(V_room,V_door,V_room_2)) :- room(V_room_2).
constant_sort(acc(V_room,V_door,V_room_2),boolean) :- room(V_room_2).
rigid(knows(V_person,V_person_2)) :- person(V_person_2).
constant_sort(knows(V_person,V_person_2),boolean) :- person(V_person_2).
rigid(passto(V_person,V_person_2)) :- person(V_person_2).
constant_sort(passto(V_person,V_person_2),boolean) :- person(V_person_2).
rigid(relatedto(V_person,V_person_2)) :- person(V_person_2).
constant_sort(relatedto(V_person,V_person_2),boolean) :- person(V_person_2).
inertialFluent(inside(V_person,V_room)) :- true.
constant_sort(inside(V_person,V_room),boolean) :- true.
inertialFluent(beside(V_door)) :- true.
constant_sort(beside(V_door),boolean) :- true.
inertialFluent(facing(V_door)) :- true.
constant_sort(facing(V_door),boolean) :- true.
inertialFluent(believeinside(V_person,V_room)) :- true.
constant_sort(believeinside(V_person,V_room),boolean) :- true.
inertialFluent(served(V_person)) :- true.
constant_sort(served(V_person),boolean) :- true.


inertialFluent(loc) :- true.
constant_sort(loc,room) :- true.
simpleFluent(open(V_door)) :- true.
constant_sort(open(V_door),boolean) :- true.
simpleFluent(visited(V_person)) :- true.
constant_sort(visited(V_person),boolean) :- true.
exogenousAction(approach(V_door)) :- true.
constant_sort(approach(V_door),boolean) :- true.
exogenousAction(gothrough(V_door)) :- true.
constant_sort(gothrough(V_door),boolean) :- true.
exogenousAction(callforopen(V_door)) :- true.
constant_sort(callforopen(V_door),boolean) :- true.
exogenousAction(goto(V_person)) :- true.
constant_sort(goto(V_person),boolean) :- true.
exogenousAction(sense(V_person,ploc(V_person_2))) :- person(V_person_2).
constant_sort(sense(V_person,ploc(V_person_2)),boolean) :- person(V_person_2).
exogenousAction(serve(V_room)) :- true.
constant_sort(serve(V_room),boolean) :- true.

h(eql(hasdoor(R,D),false)) :- {not h(eql(hasdoor(R,D),false))}0,true.
h(eql(hasdoor(o3_508,d3_508),true)) :- true.
h(eql(hasdoor(o3_510,d3_510),true)) :- true.
h(eql(hasdoor(o3_512,d3_512),true)) :- true.
h(eql(hasdoor(o3_416,d3_416),true)) :- true.
h(eql(hasdoor(l3_414a,d3_414a1),true)) :- true.
h(eql(hasdoor(l3_414a,d3_414a2),true)) :- true.
h(eql(hasdoor(l3_414a,d3_414a3),true)) :- true.
h(eql(hasdoor(s3_516,d3_5161),true)) :- true.
h(eql(hasdoor(s3_516,d3_5162),true)) :- true.
h(eql(hasdoor(l3_436,d3_4361),true)) :- true.
h(eql(hasdoor(l3_436,d3_4362),true)) :- true.
h(eql(hasdoor(o3_426,d3_426),true)) :- true.
h(eql(hasdoor(o3_428,d3_428),true)) :- true.
h(eql(hasdoor(l3_414b,d3_414b1),true)) :- true.
h(eql(hasdoor(l3_414b,d3_414b2),true)) :- true.
h(eql(hasdoor(l3_414b,d3_414b3),true)) :- true.
h(eql(hasdoor(o3_402,d3_402),true)) :- true.
h(eql(hasdoor(o3_412,d3_412),true)) :- true.
h(eql(hasdoor(o3_502,d3_502),true)) :- true.
h(eql(hasdoor(o3_404,d3_404),true)) :- true.
h(eql(hasdoor(o3_418,d3_418),true)) :- true.
h(eql(hasdoor(o3_420,d3_420),true)) :- true.
h(eql(hasdoor(o3_422,d3_422),true)) :- true.
h(eql(hasdoor(o3_430,d3_430),true)) :- true.
h(eql(hasdoor(o3_432,d3_432),true)) :- true.
h(eql(hasdoor(cor,D),true)) :- D!=d3_414a3,D!=d3_414b3.

h(eql(open(D),false),V_step) :- {not h(eql(open(D),false),V_step)}0,true.
h(eql(acc(R1,D,R2),false)) :- {not h(eql(acc(R1,D,R2),false))}0,true.

h(eql(acc(R,D,cor),true)) :- h(eql(hasdoor(R,D),true)),D!=d3_414a3,D!=d3_414b3.
h(eql(acc(R1,D,R2),true)) :- h(eql(acc(R2,D,R1),true)).
h(eql(acc(l3_414a,d3_414a3,l3_414),true)) :- true.
h(eql(acc(l3_414b,d3_414b3,l3_414),true)) :- true.

h(eql(relatedto(P1,P2),false)) :- {not h(eql(relatedto(P1,P2),false))}0,true.
h(eql(passto(P1,P2),false)) :- {not h(eql(passto(P1,P2),false))}0,true.

h(eql(relatedto(P1,P2),true)) :- h(eql(passto(P2,P1),true)).
h(eql(relatedto(P3,P1),true)) :- h(eql(passto(P1,P2),true)),h(eql(relatedto(P3,P2),true)).

h(eql(knows(alice,bob),true)) :- true.
h(eql(knows(carol,dan),true)) :- true.
h(eql(knows(P1,P2),false)) :- {not h(eql(knows(P1,P2),false))}0,true.

h(eql(beside(D1),false),V_step) :- h(eql(beside(D2),true),V_step),D2!=D1.
h(eql(facing(D2),false),V_step) :- h(eql(facing(D1),true),V_step),D2!=D1.
h(eql(beside(D),true),V_step) :- h(eql(facing(D),true),V_step).

h(eql(inside(P,R1),false),V_step) :- h(eql(inside(P,R2),true),V_step),R2!=R1.
h(eql(believeinside(P,R2),false),V_step) :- h(eql(believeinside(P,R1),true),V_step),R2!=R1.

h(eql(inside(P,R),true),V_step) :- h(eql(believeinside(P,R),true),V_step).

h(eql(open(D),false),V_step) :- {not h(eql(open(D),false),V_step)}0,true.
h(eql(visited(P),false),V_step) :- {not h(eql(visited(P),false),V_step)}0,true.

h(eql(facing(D),true),V_astep) :- occ(eql(approach(D),true),V_astep-1).
false :- occ(eql(approach(D),true),V_astep-1),h(eql(loc,R),V_astep-1),h(eql(hasdoor(R,D),false)).
false :- occ(eql(approach(D),true),V_astep-1),h(eql(facing(D),true),V_astep-1).

h(eql(loc,R),V_astep) :- h(eql(acc(R1,D,R),true)),h(eql(loc,R1),V_astep-1),occ(eql(gothrough(D),true),V_astep-1),R!=R1.
h(eql(facing(D),false),V_astep) :- occ(eql(gothrough(D),true),V_astep-1).
false :- occ(eql(gothrough(D),true),V_astep-1),h(eql(facing(D),false),V_astep-1).
false :- occ(eql(gothrough(D),true),V_astep-1),h(eql(open(D),false),V_astep-1).
false :- occ(eql(gothrough(D),true),V_astep-1),h(eql(loc,R),V_astep-1),h(eql(hasdoor(R,D),false)).

h(eql(open(D),true),V_astep) :- occ(eql(callforopen(D),true),V_astep-1).
false :- occ(eql(callforopen(D),true),V_astep-1),h(eql(facing(D),false),V_astep-1).
false :- occ(eql(callforopen(D),true),V_astep-1),h(eql(open(D),true),V_astep-1).

h(eql(visited(P),true),V_astep) :- occ(eql(goto(P),true),V_astep-1).
h(eql(served(P),true),V_astep) :- occ(eql(goto(P),true),V_astep-1).
%goto(P) causes served(P1) if relatedto(P,P1)
h(eql(served(P1),true),V_step) :- h(eql(served(P),true),V_step),h(eql(relatedto(P,P1),true)).
false :- occ(eql(goto(P),true),V_astep-1),h(eql(loc,R),V_astep-1),h(eql(believeinside(P,R),false),V_astep-1).
false :- occ(eql(goto(P),true),V_astep-1),h(eql(passto(P,P1),true)).
false :- occ(eql(goto(P),true),V_astep-1),h(eql(served(P),true),V_astep-1).
#domain room(X_Value_0).
#domain room(_NV_1).
false :- occ(eql(goto(P),true),V_astep-1),_NV_1!=R,h(eql(loc,_NV_1),V_astep-1),h(eql(believeinside(P,R),true),V_astep-1).

h(eql(believeinside(P,R),true),V_astep) :- h(eql(inside(P,R),true),V_astep-1),occ(eql(sense(P1,ploc(P)),true),V_astep-1).
false :- occ(eql(sense(P1,ploc(P)),true),V_astep-1),h(eql(visited(P1),false),V_astep-1).
false :- occ(eql(sense(P1,ploc(P)),true),V_astep-1),h(eql(visited(P1),true),V_astep-1),h(eql(knows(P1,P),false)).
false :- occ(eql(sense(P1,ploc(P)),true),V_astep-1),h(eql(believeinside(P,R),true),V_astep-1).

noconcurrency :- true.
true.
:- false.
#hide true/0.

