%%%%%%% Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const imin   = 1.
#const imax   = 10.

%%%%%%% Initial Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% assume that the initial time starts at 0
time(0).

% max. X-Coordinate of field
maxX(X) :- X=#max{S:init(object(node,_),value(at,(S,Y)))}.

% max. Y-Coordinate of field
maxY(Y) :- Y=#max{S:init(object(node,_),value(at,(X,S)))}.

% current position at start (recognised with "at, (X,Y)")
curlocation(R, X, Y, 0) :- init(object(robot,R),value(at,(X,Y))).

%%%%%%% Base Code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% distribute orders on robots randomly for now
{orderrobot(B,R):init(object(order,B),value(line,(_,_)))}=1 :- init(object(robot,R),value(at,(_,_))).

% when delivered, order is done
orderdone(B) :- occurs(object(robot, _), action(deliver, (B, _, _)), _).

% current position at time T+1 depends on movement of robot and position at time T
curlocation(R, X+A, Y+B, T+1) :- occurs(object(robot, Z), action(move, (A,B)), T), curlocation(R, X, Y, T).

% if there is no movement at time T curlocation is the same with time T+1
curlocation(R,X,Y,T+1) :- curlocation(R, X, Y, T), {occurs(object(robot, R), action(pickup, ()), T) ;
occurs(object(robot, R), action(putdown, ()), T) ;
occurs(object(robot, R), action(deliver, (1, 1, T)), T)
}=1.

% robot could do something at time T, if time limit is not reached yet
{ occurs(object(robot, R), action(move, (0,-1)), T) ;
occurs(object(robot, R), action(move, (0,1)), T) ;
occurs(object(robot, R), action(move, (-1,0)), T) ;
occurs(object(robot, R), action(move, (1,0)), T) ;
occurs(object(robot, R), action(pickup, ()), T) ;
occurs(object(robot, R), action(putdown, ()), T) ;
occurs(object(robot, R), action(deliver, (1, 1, T)), T)
}1 :- init(object(robot,R),_), time(T),T<=imax.

% if there is a time T and an action at time T, there is a time T+1.
time(T+1):-time(T),{ occurs(object(robot, Z), action(_,_), T) ;
occurs(object(robot, Z), action(deliver, (1, 1, T)), T)
}=1.

% get the number of actions of the plan for minimization
maxoccurs(T) :- T=#max{S:occurs(object(robot, _), _, S)}.

%%%%%%% Support Functions for Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robot is at picking station / fulfilles Constraints #9 & #10
isatpick(R,P,T):- curlocation(R, X, Y, T), init(object(pickingStation,P),value(at,(X,Y))).

% robot is at a shelf / fulfilles Constraint #11
isatshelf(R,S,T) :- curlocation(R, X, Y, T), init(object(shelf,S),value(at,(X,Y))).

% robot has picked up and delivered / fulfilles Constraint #7
pickdeliver(R,Z) :- occurs(object(robot, R), action(pickup, ()), T), occurs(object(robot, R), action(deliver, (1,1,C)), Z).

% robot has picked up and put down / fulfilles Constraint #8
pickput(R,Z) :- occurs(object(robot, R), action(pickup, ()), T), occurs(object(robot, R), action(putdown, ()), Z).

% tests if a picking station and a needed shelf are on the same node so there is no pickup and putdown needed for the shelf / fulfilles Constraint #7
shelfatpick(P,S) :- init(object(pickingStation,P),value(at,(X,Y))), init(object(shelf,S),value(at,(X,Y))).
onlydeliver(R) :- shelfatpick(P,S), occurs(object(robot, R), action(deliver, (1,1,C)), Z).

%%%%%%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% #1 no movements over border
:- curlocation(R, X, 1, T), occurs(object(robot, R), action(move, (0,-1)), T).
:- curlocation(R, X, Y, T), occurs(object(robot, R), action(move, (0,1)), T), maxY(Y).
:- curlocation(R, X, Y, T), occurs(object(robot, R), action(move, (1,0)), T), maxX(X).
:- curlocation(R, 1, Y, T), occurs(object(robot, R), action(move, (-1,0)), T).

% #2 a robot has only one order and an order has only one robot
:- orderrobot(O,R), orderrobot(A,B), O==A, R!=B.
:- orderrobot(O,R), orderrobot(A,B), O!=A, R==B.

% #3 there is no unfinished order
:- init(object(order,B),value(line,(_,_))), not orderdone(B).

% #4 robots can only pickup one time
:- occurs(object(robot, R), action(pickup, ()), T), occurs(object(robot, R), action(pickup, ()), Z), T!=Z.

% #5 robots can only putdown one time
:- occurs(object(robot, R), action(putdown, ()), T), occurs(object(robot, R), action(putdown, ()), Z), T!=Z.

% #6 robots can only deliver one time
:- occurs(object(robot, R), action(deliver, (1,1,C)), Z),occurs(object(robot, R), action(deliver, (1,1,D)), T),T!=Z.

% #7 if there is deliver, there is a pickup before, except the needed shelf is at the same node as the picking station
:- occurs(object(robot, R), action(deliver, (1,1,C)), Z), occurs(object(robot, R), action(pickup, ()), T), Z<T.
:- occurs(object(robot, R), action(deliver, (1,1,C)), Z), not pickdeliver(R,Z), not onlydeliver(R).

% #8 if there is a putdown, there has to be a pickup before
:- occurs(object(robot, R), action(putdown, ()), Z), not pickput(R,Z).
:- occurs(object(robot, R), action(putdown, ()), Z), occurs(object(robot, R), action(pickup, ()), T), Z<T.

% #9 robots can only deliver at picking stations
:- orderrobot(B,R), init(object(order,B),value(pickingStation,P)), init(object(pickingStation,P),value(at,(X,Y))), occurs(object(robot, R), action(deliver, (1, 1, _)), T), not isatpick(R,P,T) .

% #10 putdown only at picking station
:- orderrobot(B,R), init(object(order,B),value(pickingStation,P)), init(object(pickingStation,P),value(at,(X,Y))), occurs(object(robot, R), action(putdown, (,)), T), not isatpick(R,P,T) .

% #11 pickup only when robot at correct location & shelf needed for order
:- orderrobot(B,R), init(object(order,B),value(line,(G,H))), init(object(shelf,S),value(at,(X,Y))), init(object(product,G),value(on,(X,Y))), occurs(object(robot, R), action(pickup, ()), T), not isatshelf(R,S,T).

% #12 there is no putdown, between a pickup and a deliver
:- occurs(object(robot, R), action(deliver, (1,1,C)), Z), occurs(object(robot, R), action(pickup, ()), S), occurs(object(robot, R), action(putdown, ()), T), T<Z, S<T.

%%%%%%% Minimize %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% minimize count of steps needed for all plans
#minimize {T: maxoccurs(T)}.

%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#show occurs/3.
