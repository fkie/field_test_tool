INSERT INTO ito_reason VALUES ( 1 , 'safety' , 'Safety' , 'Safety stop, initiated by an inhabitant of the vehicle.' , 'F' , '255,0,0' );
INSERT INTO ito_reason VALUES ( 2 , 'maintenance' , 'Maintenance' , 'Maintenance stop, a stop initiated be the performer to fix code.' , 'T' , '255,240,0' ) ;
INSERT INTO ito_reason VALUES ( 3 , 'planner' , 'Planner' , 'Planner stop, a stop caused by a timeout of the planer.' , 'F' , '255,120,0' ) ;
INSERT INTO ito_reason VALUES ( 4 , 'apparatus' , 'Apparatus' , 'Apparatus stop, a stop that is initiated by the test admin to fix the test apparatus.' , 'T' , '70,0,255' ) ;
INSERT INTO ito_reason VALUES ( 5 , 'non_stop' , 'Non-Stop' , 'Non-Stop, indicates that no stop was performed. Ignore this ITO.' , 'F' , '10,120,10' ) ;
INSERT INTO ito_reason VALUES ( 6 , 'stereo' , 'Stereo test' , 'Stereo test stop, a stop initiated to perform a test of the stereo accuracy.' , 'F' , '0,170,255' ) ;
INSERT INTO ito_reason VALUES ( 7 , 'goal' , 'Goal reached' , 'Goal reached, a stop because the vehicle reached an (intermediate) goal.' , 'F' , '0,0,139' ) ;
INSERT INTO ito_reason VALUES ( 8 , 'end_shift' , 'End of shift' , 'End of shift indicates the stop was caused because the time for the shift was up.' , 'F' , '0,0,220' ) ;
INSERT INTO ito_reason VALUES ( 9 , 'manual_transit' , 'Manual transit' , 'Manual transit indicates that the vehicle was stopped to drive it manually through a dangerous area. This time and distance is not counted against the performer.' , 'T' , '100,100,100' ) ;
INSERT INTO ito_reason VALUES ( 10 , 'other' , 'Other' , 'Other reasons.' , 'F' , ' 180,180,180' ) ;
INSERT INTO ito_reason VALUES ( 11 , 'unassigned' , 'Unassigned' , 'Unassigned (deprecated).' , 'F' , '80,80,80' ) ;
INSERT INTO ito_reason VALUES ( 12 , 'unexpected' , 'Unexpected' , 'Unexpected behavior, such as touching a barrel.' , 'F' , '200,200,0' ) ;

INSERT INTO segment_type VALUES ( 1 , 'ito' , 'ITO' , 'ITO - inhabited take over.' , '255,0,0' ) ;
INSERT INTO segment_type VALUES ( 2 , 'auto' , 'AUTO' , 'Autonomous.' , '0,255,0' ) ;

INSERT INTO weather VALUES ( 1 , 'sunny' , 'sunny' , 'Mainly sunny, less than 30 percent of the sky is covered by clouds' , 'sun79' ) ;
INSERT INTO weather VALUES ( 2 , 'cloudy' , 'cloudy' , 'Mainly cloudy, between 30 and 95 percent of the sky is covered by clouds' , 'cloudy19' ) ;
INSERT INTO weather VALUES ( 3 , 'overcast' , 'overcast' , 'More than 95 percent of the sky is covered by clouds' , 'two183' ) ;
INSERT INTO weather VALUES ( 4 , 'foggy' , 'foggy' , 'Reduced visibility due to fog to less than one kilometer' , 'fog8' ) ;
INSERT INTO weather VALUES ( 5 , 'rainy' , 'rainy' , '-' , 'rain27' ) ;
INSERT INTO weather VALUES ( 6 , 'snowy' , 'snowy' , '-' , 'snowing' ) ;