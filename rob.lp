%%%%%%% Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const imin   = 1.
#const imax   = 2.

%%%%%%% Initial Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% assume that the initial time starts at 0
time(0).

% max. and min. X-Coordinate
horizontal(X) :- init(object(node,_),value(at,(X,Y))), init(object(node,_),value(at,(A,B))), X<=A.
maxX(X) :- X=#max{S:horizontal(S)}.
minX(X) :- X=#min{S:horizontal(S)}.

% max. and min. Y-Coordinate 
vertical(Y) :- init(object(node,_),value(at,(X,Y))), init(object(node,_),value(at,(A,B))), Y>=B.
maxY(Y) :- Y=#max{S:vertical(S)}.
minY(Y) :- Y=#min{S:vertical(S)}.

%%%%%%% Base Code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% current position at start (recognised with "at, (X,Y)")
curlocation(R, X, Y, 0) :- init(object(robot,R),value(at,(X,Y))).
% current position at time T depends on movement of robot and position at time T-1
curlocation(R, X+A, Y+B, T) :- occurs(object(robot, Z), action(move, (A,B)), T), curlocation(R, X, Y, T-1).

% robot could do something at time T, if time limit is not reached yet
{ occurs(object(robot, R), action(move, (0,-1)), T) ;
occurs(object(robot, R), action(move, (0,1)), T) ;
occurs(object(robot, R), action(move, (-1,0)), T) ;
occurs(object(robot, R), action(move, (1,0)), T) ;	
occurs(object(robot, R), action(pickup, ()), T) ;
occurs(object(robot, R), action(putdown, ()), T) ;
occurs(object(robot, R), action(deliver, (1, 1, T)), T)
}1 :- init(object(robot,R),_), time(T),T<=imax.

% if there is a time T and an action at time T, th	ere is a time T+1.
time(T+1):-time(T),{ occurs(object(robot, Z), action(move, (0,-1)), T) ;
occurs(object(robot, Z), action(move, (0,1)), T) ;
occurs(object(robot, Z), action(move, (-1,0)), T) ;
occurs(object(robot, Z), action(move, (1,0)), T) ;	
occurs(object(robot, Z), action(pickup, ()), T) ;
occurs(object(robot, Z), action(putdown, ()), T) ;
occurs(object(robot, Z), action(deliver, (1, 1, T)), T)
}=1.

%%%%%%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% no movements over border
:- curlocation(R, X, Y, T), occurs(object(robot, R), action(move, (0,-1)), T+1), minY(Y). 
:- curlocation(R, X, Y, T), occurs(object(robot, R), action(move, (0,1)), T+1), maxY(Y). 
:- curlocation(R, X, Y, T), occurs(object(robot, R), action(move, (1,0)), T+1), maxX(X). 
:- curlocation(R, X, Y, T), occurs(object(robot, R), action(move, (-1,0)), T+1), minX(X). 


%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%#show occurs/3.
#show curlocation/4.

