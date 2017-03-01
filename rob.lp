%%%%%%% Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const imin   = 1.
#const imax   = 2.

%%%%%%% Initial Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% assume that the initial time starts at 0
time(0).

% max. X-Coordinate
horizontal(X) :- init(object(node,_),value(at,(X,Y))), init(object(node,_),value(at,(A,B))), X<=A.
maxX(X) :- X=#max{S:horizontal(S)}.

% max. Y-Coordinate 
vertical(Y) :- init(object(node,_),value(at,(X,Y))), init(object(node,_),value(at,(A,B))), Y>=B.
maxY(Y) :- Y=#max{S:vertical(S)}.

%%%%%%% Base Code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% distribute orders on robots randomly for now
{orderrobot(B,R):init(object(order,B),value(line,(_,_)))}=1 :- init(object(robot,R),value(at,(_,_))).

% when delivered, order is done maybe add current time
orderdone(B) :- occurs(object(robot, _), action(deliver, (B, _, _)), _).

% current position at start (recognised with "at, (X,Y)")
curlocation(R, X, Y, 0) :- init(object(robot,R),value(at,(X,Y))).

% current position at time T+1 depends on movement of robot and position at time T
curlocation(R, X+A, Y+B, T+1) :- occurs(object(robot, Z), action(move, (A,B)), T), curlocation(R, X, Y, T).

% if there is no movement at time T curlaction is the same with time T+1
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

% robot is at picking station
isatpick(R,P,T):- curlocation(R, X, Y, T), init(object(pickingStation,P),value(at,(X,Y))).

% robot has picked up
haspicked(R) :- occurs(object(robot, R), action(pickup, ()), T), occurs(object(robot, R), action(deliver, (1,1,C)), Z),T<Z.

%%%%%%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% no movements over border
:- curlocation(R, X, 1, T), occurs(object(robot, R), action(move, (0,-1)), T).
:- curlocation(R, X, Y, T), occurs(object(robot, R), action(move, (0,1)), T), maxY(Y). 
:- curlocation(R, X, Y, T), occurs(object(robot, R), action(move, (1,0)), T), maxX(X). 
:- curlocation(R, 1, Y, T), occurs(object(robot, R), action(move, (-1,0)), T).

% a robot has only one order and an order has only one robot
:- orderrobot(O,R), orderrobot(A,B), O==A, R!=B.
:- orderrobot(O,R), orderrobot(A,B), O!=A, R==B.

% there is no unfinished order 
:- init(object(order,B),value(line,(_,_))), not orderdone(B).

% there is no putdown before picking up
:- occurs(object(robot, R), action(pickup, ()), T), occurs(object(robot, R), action(putdown, ()), Z), Z<T.

% if there is a putdown there has to be a picking up before
:- time(T), occurs(object(robot, R), action(putdown, ()), Z), not occurs(object(robot, R), action(pickup, ()), T).

% there is no deliver before pickung up
:- occurs(object(robot, R), action(pickup, ()), T), occurs(object(robot, R), action(deliver, (1,1,C)), Z), Z<T.

% if there is a deliver there has to be a picking up before and no putdown inbetween (inbetween not finished)
:- init(object(robot,R),value(at,(X,Y))), not haspicked(R).

% robots can only deliver at picking stations
:- orderrobot(B,R), init(object(order,B),value(pickingStation,P)), init(object(pickingStation,P),value(at,(X,Y))), occurs(object(robot, R), action(deliver, (1, 1, _)), T), not isatpick(R,P,T) .

% pickup only when robot at correct location & shelf needed for order


% putdown only at correct location

% it is not allowed to create a plan that does not deliver the item (orderdone needs to be true)

% deliver if robot is at correct picking station instead of putdown before ???

% minimize count of steps needed for all plans
%#minimize {T: occurs(object(robot, _), action(deliver, (_, _, _)), T)}.

%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#show occurs/3.
%#show orderrobot/2.
%#show curlocation/4.

