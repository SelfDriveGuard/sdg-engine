/***************************AVScenarios.g4*************************************
 * 
 * This is the grammar coming from BNF rules written within antlr4.
 * Please see https://antlr.org for detail description about antlr4.
 * See detailed and clean description about the grammar:docs/scenest-grammar.md
 *
 *****************************************************************************/



// TODO:Under development.(2020.10.8)
// NOTICE: Due to that we want to generate python3 target, we must avoid parsing rules names
// same as the python3 keyword and antlr4 builtin variables, so we add a '_' 
// in every world conflict with python3 keyword,
// for example: map->map_,state->state_

grammar AVScenarios;

// start rule.
scenarios:assignment_statements EOF?    # entry
		 ;
scenario:'CreateScenario' '{' map_parameter ';'
		   ego_parameter';'
		   npc_vehicles_parameter';'
		   pedestrians_parameter';'
		   obstacles_parameter';'
		   env_parameter';'
		   traffic_parameter';'
		   '}'                          #create_scenario
		   ;
npc_vehicles_parameter:identifier    #npc_var
					   |npc_vehicles     #npc_npc
					   |'{''}'           #npc_empty
					   ;
pedestrians_parameter:identifier     #pedestrians_var
					 |pedestrians       #pedestrians_ped
					 |'{''}'            #pedestrians_empty
					 ;
obstacles_parameter:identifier        #obstacles_var
				   |obstacles            #obstacles_obs
				   |'{''}'               #obstacles_empty
				   ;
traffic_parameter:identifier          #traffic_var
				 |traffic                #traffic_tra
				 |'{''}'                 #traffic_empty
				 ;
map_parameter:'load''(' map_name')'       #map_load_name
			 |'load' '('identifier')' #map_load_var
			 ;
map_name:String                         #map_name_str
		;
/* parameter_list_ego:describe the parameters explicitly
 * or all parameters are generated randomly.
 * */
ego_parameter:ego_vehicle               #ego_ego_vehicle
			 |identifier            #ego_ego_var
			 ;
ego_vehicle:'AV''('parameter_list_ego')' #ego_av
			;
/* first <state> is the initial state of the ego vehicle;
 * second <state> is the target state of the ego vehicle;
 * without <vehicle_type>, use the default one.
 * */
parameter_list_ego:state_parameter','state_parameter(','vehicle_type_parameter)? #par_list_ego_
				  ;
state_parameter:state_                 #state_state
				|identifier     #state_state_var
				;
state_:'('position_parameter')'                                         #state_position
	 | '('position_parameter','heading_parameter?(','speed_parameter)?')'   #state_position_heading_speed
	 ;
position:coordinate_frame?coordinate   #pos_coor_coor
		;
/* IMU:vehicle coordinate system, right-forward-up, origin is the
 * position of the vehicle.
 * ENU:east-north-up, map origin is the origin of the coordinate;
 * WGS84:world geodetic system;
 * default coordinate frame:ENU.
 * */
coordinate_frame:'IMU'                     #coor_imu
				|'ENU'                     #coor_enu
				|'WGS84'                   #coor_wgs84
				;
position_parameter:position            #pos_pos
				  |identifier        #pos_pos_var
				  ;
speed_parameter:speed                  #speed_speed
			   |identifier            #speed_speed_var
			   ;
speed:real_value                        #speed_rv
	 ;
real_value:Signal?non_negative_real_value     #rv
		  ;
non_negative_real_value:(float_value|number_value)    #non_negative_rv
					   ;
float_value:Non_negative_value                        #non_negative_float
		   ;
/// XXX: add extra 0 and 1 to avoid conflict lexer.
number_value:Non_negative_number                      #non_negative_number
			| '0'                                     #non_negative_conflict_0
			| '1'                                     #non_negative_conflict_1
			;
/* <laneID>-><real_value>:the distance of the point to the start point
 * of <laneID> is <real_value>.
 * */
coordinate:'('real_value','real_value(',' Signal real_value)?')'      #coor_rv_rv
			| laneID_parameter'->'real_value  #coor_laneID_rv
			;

laneID_parameter:identifier         #laneID_laneID_var
				|laneID                #laneID_laneID
				;
/* Lane is a string of such format:
 * "road_id.lane_id" or ".lane_id",
 * road_id and lane_id must be integers
 * for example: "1.5",".4"
 * */
laneID:String                    #laneID_str
	  ;
heading_parameter:identifier          #head_var
				 |heading                #head_heading
				 ;
/* default value: the direction to the lane where the position is on.
 * */
heading:real_value unit('related to' direction)?     #head_rv
	   ;
unit:'deg'                                           #unit_deg
	|'rad'                                           #unit_rad
	;
direction:predefined_direction                       #direction_pre
		  |customized_direction                      #direction_cus
		  ;
predefined_direction:'lane'laneID_parameter          #pre_lane
					 |'EGO'                          #pre_ego
					 ;
/* default coordinate frame is ENU.
 * */
customized_direction:'('real_value','real_value(','coordinate_frame)?')'    #cus_rv_rv
					| '('real_value','real_value','Signal real_value(','coordinate_frame)?')'    #cus_rv_rv_rv
					;
vehicle_type_parameter:identifier            #vehicle_vehicle_type_var
				 |vehicle_type                            #vehicle_vehicle_type
				 ;
/* currently,we do not consider <material>.
 * */
vehicle_type:'('type_parameter')'                                     #vehicle_type_
			|'('type_parameter','color_parameter?/*(','material)?*/')'  #vehicle_type_color
			;
type_parameter:identifier    #type_var
			  |type_            #type_type_
			  ;
type_:specific_type        #type_specific
	 |general_type         #type_general
	 ;
specific_type:String                     #specific_str
			 ;
/* model of the vehicle.
 * */
general_type:'car'    #general_car
			|'bus'    #general_bus
			|'Van'    #general_van
			|'truck'  #general_truck
			|'bicycle' #general_bicycle
			|'motorbicycle' #general_motorbicycle
			|'tricycle'     #general_tricycle
			;
color_parameter:identifier        #color_var
			   |color                #color_color
			   ;
color:color_list            #color_color_list
	 |rgb_color             #color_rgb_color
	 ;
color_list:'red'            #color_red
		  |'green'          #color_green
		  |'blue'           #color_blue
		  |'black'          #color_black
		  |'white'          #color_white
		  ;
/// rgb_color must be a value rgb value.
rgb_color:Rgb_color         #rgb_rgb
		 ;
npc_vehicles:'{'multi_npc_vehicles'}'       #npc
			;
multi_npc_vehicles:npc_vehicle_parameter     #multi_npc
				 |multi_npc_vehicles','npc_vehicle_parameter     #multi_multi_npc
				 ;
npc_vehicle:'Vehicle''('parameter_list_npc')'     #npc_vehicle_par
		   ;
npc_vehicle_parameter:npc_vehicle                   #npc_npc_vehicle
					 |identifier        #npc_npc_vehicle_var
					 ;
/* first <state> is the initial state of a vehicle;
 * second <state> is the target state of a vehicle;
 * default motion:uniform form motion along paths.
 * */
parameter_list_npc:state_parameter                                #par_npc_state
				   |state_parameter ','vehicle_motion_parameter   #par_npc_state_vehicle
				   |state_parameter','vehicle_motion_parameter?','state_parameter?(','vehicle_type_parameter)?  #par_npc_state_vehicle_state
				   ;
vehicle_motion_parameter:vehicle_motion                 #vehicle_vehicle_motion
						|identifier      #vehicle_vehicle_motion_var
						;
vehicle_motion:uniform_motion               #vehicle_motion_uniform
			  |waypoint_motion              #vehicle_motion_waypoint
			  ;
/* move with the given speed in <state>.
 * */
uniform_motion:uniform_index'('state_parameter')'        #uniform
			  ;
uniform_index:'uniform'      #uniform_uniform
			 |'Uniform'      #uniform_Uniform
			 |'U'            #uniform_U
			 |'u'            #uniform_u
			 ;
waypoint_motion:waypoint_index'('state_list_parameter')'      #waypoint
			   ;
state_list_parameter:identifier      #state_state_list_var
					|state_list         #state_state_list
					;
state_list:'('multi_states')' #state_list_multi
		  ;
multi_states:multi_states','state_parameter    #multi_states_par_state
			|state_parameter                   #multi_states_par
			;
waypoint_index:'Waypoint'   #waypoint_Waypoint
			  |'W'          #waypoint_W
			  |'WP'         #waypoint_WP
			  |'waypoint'   #waypoint_waypoint
			  |'w'          #waypoint_w
			  |'wp'         #waypoint_wp
			  ;
pedestrians:'{'multiple_pedestrians'}'   #pedestrians_multi
		   ;
multiple_pedestrians:pedestrian_parameter   #multi_pedestrian
					|multiple_pedestrians','pedestrian_parameter   #multi_multi_pedestrian
					;
pedestrian_parameter:pedestrian                 #pedestrian_pedestrian
					|identifier      #pedestrian_pedestrian_var
					;
pedestrian:'Pedestrian''('parameter_list_ped')'   #pedestrian_par
		  ;
/* default motion:uniform motion along the crosswalk with the same direction
 * of the nearest lane parallel with the crosswalk.
 * */
parameter_list_ped:state_parameter                                       #par_ped_state
				   |state_parameter','pedestrian_motion_parameter        #par_ped_state_ped
				   |state_parameter','pedestrian_motion_parameter?','state_parameter?(','pedestrian_type_parameter)?   #par_ped_state_ped_state
				   ;
pedestrian_motion_parameter:pedestrian_motion            #pedestrian_motion_pedestrian
						   |identifier                #pedestrian_motion_pedestrian_var
						   ;
pedestrian_motion:uniform_motion    #pedestrian_uniform
				 |waypoint_motion   #pedestrian_waypoint
				 ;
pedestrian_type_parameter:pedestrian_type             #pedestrian_pedestrian_type
						 |identifier  #pedestrian_pedestrian_type_var
						 ;
pedestrian_type:'('height_parameter','color_parameter')'    #pedestrian_type_height_color
				;
height_parameter:identifier  #height_var
				|height         #height_height
				;
height:real_value                 #height_rv
	  ;
obstacles:'{'multiple_obstacles'}'      #obstacles_multi
		 ;
multiple_obstacles:obstacle_parameter                            #obstacles_obstacle
				  |multiple_obstacles','obstacle_parameter       #obstacles_multi_obstacle
				  ;
obstacle_parameter:obstacle                                      #obstacle_obstacle
				  |identifier                           #obstacle_obstacle_var
				  ;
obstacle:'Obstacle''('parameter_list_obs')'                      #obstacle_para
		;
parameter_list_obs:position_parameter(','shape_parameter)?       #par_position_shape
				  ;
shape_parameter:identifier                                 #shape_shape_var
				|shape                                           #shape_shape
				;
shape:sphere              #shape_sphere
	 |box                 #shape_box
	 |cone                #shape_cone
	 |cylinder            #shape_cylinder
	 ;
sphere:'(''sphere'','non_negative_real_value')'     #sphere_sphere
	  ;
box:'(''box'','non_negative_real_value','non_negative_real_value','non_negative_real_value')'  #box_box
   ;
cone:'(''cone'','non_negative_real_value','non_negative_real_value','non_negative_real_value')'  #cone_cone
	;
cylinder:'(''cylinder'','non_negative_real_value','non_negative_real_value','non_negative_real_value')'  #cylinder_cylinder
		;
env_parameter:identifier                              #env_var
			 |env                                         #env_env
			 // NOTICE:add a default empty environment.
			 |'{''}'                                      #env_empty
			 ;
env:'Environment''('parameter_list_env')'                 #env_par
   ;
parameter_list_env:time_parameter','weather_parameter               #par_time_weather
				  ;
weather_parameter:identifier            #weather_var
				 |weather                 #weather_wtr
				 ;
time_parameter:time                         #time_time
			  |identifier              #time_time_var
			  ;
time:Time                                  #time_Time
	;
weather:'{'multi_weathers'}'            #weathers
	   ;
multi_weathers:weather_statement_parameter                       #weathers_weather
			  |multi_weathers','weather_statement_parameter      #weathers_multi_weather
			  ;
weather_statement_parameter:identifier                #weather_weather_var
						   |weather_statement            #weather_weather
						   ;
weather_statement:kind':'weather_continuous_index_parameter       #weather_continuous
				 |kind':'weather_discrete_level_parameter        #weather_discrete
				 ;
kind:'sunny'                             #kind_sunny
	|'rain'                              #kind_rain
	|'snow'                              #kind_snow
	|'fog'                               #kind_fog
	|'wetness'                           #kind_wetness
	;
/// float_value must 0.0-0.9 or 1.0
weather_continuous_index_parameter:float_value                   #weather_continuous_value
								  |identifier                     #weather_continuous_var
								  ;
weather_discrete_level_parameter:weather_discrete_level                #weather_discrete_level_par
								|identifier                            #weather_discrete_var
								;
weather_discrete_level:'light'                 #weather_discrete_light
					  |'middle'                #weather_discrete_middle
					  |'heavy'                 #weather_discrete_heavy
					  ;
traffic:'{'traffic_statement'}'                          #traffic_traffic
	   ;
traffic_statement:intersection_traffic','lane_traffic     #traffic_stmt
				 ;
intersection_traffic:meta_intersection_traffic_parameter(','meta_intersection_traffic_parameter)* #intersection
					;
meta_intersection_traffic_parameter:identifier         #meta_intersection_meta_var
									|meta_intersection_traffic        #meta_intersection_meta
									;
meta_intersection_traffic:'Intersection''('intersection_ID_parameter','
						  ('0'|'1')','('0'|'1')','('0'|'1')')'   #meta_intersection_intersection
						 ;
intersection_ID_parameter:intersection_ID                #intersection_intersection
						 |identifier     #intersection_intersection_var
						 ;
intersection_ID:Signal? number_value              #intersection_signal
			   ;
lane_traffic:speed_limitation_parameter                  #lane_speed_limit
			|lane_traffic','speed_limitation_parameter   #lane_lane_speed_limit
			;
speed_limitation_parameter:speed_limitation        #speed_limit
						  |identifier           #speed_limit_var
						  ;
speed_limitation:'SpeedLimit''('laneID_parameter','speed_range_parameter')'  #speed_limit_speed_limit
				;
speed_range_parameter:identifier         #speed_range_var
					 |speed_range           #speed_range_speed
					 ;
speed_range:'('non_negative_real_value','non_negative_real_value')'   #speed_range_value
		   ;

// assertions.
/// identifier denotes the scenario.
trace_assignment:'Trace' trace_identifier'=''EXE''('identifier')'          #trace_scenario
				;
trace_identifier:identifier									#trace_id
				;
detection_assertion:trace_identifier'|=''G'detection_statement     #trace_detection
				   ;
detection_statement:single_detection							#detection_single
				   |detection_statement'&'single_detection       #detection_detection_single
				   ;
single_detection:agent_detection							#single_agent
				|traffic_detection							#single_traffic
				;
agent_detection:agent_visible_statement'&'agent_error_statement		#agent_visible
			   ;
agent_visible_statement:agent_ground_distance_parameter'<='sensing_range		#agent_visible_assert
					   ;
agent_ground_distance_parameter:identifier				#agent_id
                               |agent_ground_distance	#agent_par
                               ;
agent_ground_distance:'dis''('ego_state_parameter','agent_ground_truth_parameter')'		#agent_ground
					 ;
ego_state_parameter:identifier		#ego_state_id
                   |ego_state		#ego_state_par
                   ;
ego_state:trace_state'[''ego'']'			#ego_state_ego
		 ;
agent_ground_truth_parameter:identifier		#agent_ground_truth_id
                            |agent_ground_truth	#agent_ground_truth_par
                            ;
/// identifier must be a npc vehicle,a pedestrian,or an obstacle
agent_ground_truth:trace_state'[''truth'']''['identifier']'		#agent_ground_id
				  ;
trace_state:trace_identifier'['number_value']'			#trace_number
		   ;
sensing_range:non_negative_real_value					#sensing_value
			 ;
agent_error_statement:agent_error_parameter'<='error_threshold		#agent_error_assert
					 ;
error_threshold:non_negative_real_value							#error_value
			   ;
agent_error_parameter:identifier			#agent_error_id
                     |agent_error			#agent_error_par
                     ;
agent_error:'diff''('agent_state_parameter','agent_ground_truth_parameter')'	#agent_error_stmt
		   ;
agent_state_parameter:identifier			#agent_state_id
                     |agent_state			#agent_state_par
                     ;
/// identifier must be a npc,a pedestrian,or an obstacle
agent_state:trace_state'[''perception'']''['identifier']'		#agent_state_trace
		   ;
/// XXX: <trace_detection_right> is only helpful for parsing
traffic_detection:trace_state'[''perception'']''[''traffic'']'trace_detection_right   #traffic_detection_assert
                 ;
trace_detection_right:'=='trace_state'[''truth'']''[''traffic'']'	#traffic_detection_assert_right
				 ;
// safety assertion
safety_assertion:trace_identifier'|=''G'safety_statement	#trace_safety
				;
safety_statement:single_safety_statement						#safety_single
				|safety_statement'&'single_safety_statement		#safety_single_single
				;
single_safety_statement:agent_detection'&'agent_safety_statement	#single_safety
					   ;
agent_safety_statement:'dis''('ego_state_parameter','agent_state_parameter')''>='safety_radius	#agent_safety
					  ;
safety_radius:float_value		#safety_radius_value
			 ;
// intersection statement
intersection_assertion:trace_identifier'|=''G'red_light_statement	#trace_intersection
				      ;
red_light_statement:'('traffic_detection'&'red_light')'red_light_statement_right  #red_light_stmt
                   ;
/// XXX: <red_light_statement_right> is only helpful for parsing
red_light_statement_right:'->''(''~'ego_speed'U''('traffic_detection'&'green_light')'')'  #red_light_stmt_right
					     ;
red_light:trace_state'[''traffic'']''==''red'	#red
		 ;
green_light:trace_state'[''traffic'']''==''green'	#green
		   ;
ego_speed:'norm''('ego_velocity')'		#ego_speed_value
		 ;
ego_velocity:coordinate			#ego_velocity_value
			;
// lane statement
speed_constraint_assertion:trace_identifier'|=''G' speed_statement		#trace_speed
						 ;
speed_statement:'('traffic_detection'&'speed_limitation_checking'&'speed_violation')'speed_statement_right
                    #speed_assert
               ;
/// XXX: <speed_statement_right> is only helpful for parsing
speed_statement_right:'->''F''[''0'','time_duration']''~'speed_violation   #speed_assert_right
			   ;
speed_limitation_checking:trace_state'[''traffic'']''=='speed_range_parameter	#speed_checking
						 ;
speed_violation:speed_parameter'<'trace_state'[''traffic'']''[''0'']'	#speed_violation_stmt0
				|speed_parameter'<'trace_state'[''traffic'']''[''1'']'	#speed_violation_stmt1
			   ;
time_duration:non_negative_real_value		#time_duration_value
			 ;


// statements.
assignment_statements:(assignment_statement';')*          #assigns
					 ;
assignment_statement:identifier'='scenario             #assign_scenario
					/// this rule may refer to <Variable>::= '='<height>
					/// <Variable>::= '='<speed>
					/// <Variable>::= '='<weather_continuous_index>
					/// <Variable>::= '='<intersection_ID>.
					|identifier'='real_value           #assign_rv
					/// this rule may refer to <map_name>::= '='<string>
					/// <type_>::= '='<specific_type>
					/// <Variable>::= '='<laneID>
					|identifier'='String               #assign_str
					|identifier'='ego_vehicle          #assign_ego
					/// this rule may refer to
					/// <state>::= '=''('<position>')'
					/// <vehicle_type>::= '='<type_>
					/// <state_list>::='('<state>')'
					|identifier'=''('identifier')'     #assign_variable
					|identifier '=''('identifier','identifier')'       #assign_name_two_variables
					/// this rule may refer to
					/// <state>::= '=''('<position>','<heading>','<speed>')'
					/// <state_list>::='('<state>','<state>','<state>')'
					|identifier'=''('identifier','identifier','identifier')'            #assign_name_three_variables
					|identifier'='state_								#assign_state
					|identifier'='vehicle_type						#assign_vehicle_type
					|identifier'='state_list				#assign_state_list
					|identifier'='pedestrian_type			#assign_pedestrian_type
					/// this rule may refer to
					/// <position>::= '='coordinate_frame?'('real_value','real_value')'
					/// <Variable>::= '='<speed_range>
					|identifier'='coordinate_frame?'('real_value','real_value(','Signal real_value)?')'       #assign_rv_rv
					|identifier'='coordinate_frame?laneID_parameter'->'real_value      #assign_lane_rv
					|identifier'='heading                     #assign_heading
					|identifier'='general_type                #assign_general_type
					|identifier'='color                       #assign_color
					|identifier'='npc_vehicle                 #assign_npc
					/// the next two rules may refer to
					/// <Variable>::= '='<vehicle_motion>
					/// <Variable>::= '='<pedestrian_motion>
					|identifier'='uniform_motion         #assign_uniform_motion
					|identifier'='waypoint_motion        #assign_waypoint_motion
					|identifier'='state_list             #assign_state_list
					/// XXX: notice the next rule may refer to
					/// multiple <pedestrian>s
					/// multiple <npc_vehicle>s
					/// multiple <obstacle>s
					/// <weather>
					/// <traffic>
					/// therefore of many variable will not fall 
					/// into next five rules.
					|identifier'=''{'identifier(','identifier)*'}'   #assign_variables
					|identifier'='pedestrians                #assign_pedestrians
					|identifier'='npc_vehicles               #assign_npcs
					|identifier'='obstacles                  #assign_obstacles
					|identifier'='weather                    #assign_weather
					|identifier'='traffic                    #assign_traffic
					|identifier'='pedestrian                 #assign_ped
					|identifier'='obstacle                   #assign_obs
					|identifier'='shape                      #assign_shape
					|identifier'='env                        #assign_env
					|identifier'='time                       #assign_time
					|identifier'='weather_statement          #assign_weather_stmt
					|identifier'='weather_discrete_level     #assign_weather_discrete
					|identifier'='meta_intersection_traffic  #assign_intersection
					|identifier'='speed_limitation           #assign_speed_limit
					|trace_assignment                        #assign_trace
					/// The identifier may be detection_assertion safety_assertion
					/// intersection_assertion or speed_constraint_assertion
					|trace_identifier'|=''G'identifier		 #assign_assertion			
					|detection_assertion					 #assign_detection
					|safety_assertion						 #assign_safety
					|intersection_assertion			      	 #assign_intersection_assert
					|speed_constraint_assertion			     #assign_speed_constraint
					|identifier'='detection_statement		 #assign_detection_id
					|identifier'='safety_statement			 #assign_safety_id
					|identifier'='red_light_statement        #assign_intersection_assert_id
					|identifier'='speed_statement			 #assign_speed_constraint_id
					|identifier'='agent_ground_truth         #assign_agent_ground
					|identifier'='agent_ground_distance      #assign_agent_ground_dis
					|identifier'='ego_state                  #assign_ego_state
					|identifier'='agent_error                #assign_agent_error
					|identifier'='agent_state                #assign_agent_state
					;


/// XXX: The following rule allows keywords as identifiers.
identifier:Variable_name
		  |'CreateScenario'|'load'|'AV'|'IMU'|'ENU'|'WGS84'
		  |'deg'|'rad'|'lane'|'EGO'|'car'|'bus'|'Van'|'truck'
		  |'bicycle'|'motorbicycle'|'tricycle'|'Vehicle'|'uniform'|'Uniform'
		  |'U'|'u'|'Waypoint'|'w'|'wp'|'W'|'WP'|'waypoint'|'Pedestrian'
		  |'Obstacle'|'Environment'|'Intersection'|'SpeedLimit'|'EXE'|'G'|'dis'|'diff'
		  |'truth'|'perception'|'traffic'|'U'|'F'|'norm'
		  ;




// lexer.


String:'"'Input_character+?'"'
	  ;
Signal:'-'
	  |'+'
	  ;
fragment Input_character:[a-zA-Z0-9]
						 |Symbol
						 ;
fragment Symbol :'`'|'~'|'!'|'@'|'#'|'$'|'%'|'^'|'&'|'*'|'('|')'
				|'_'|'-'|'+'|'='|'\\'|'|'|'['|']'|'{'|'}'|';'
				|':'|'\''|'"'|'/'|'?'|'<'|'>'|','|'.'|' '
				;
Variable_name:[a-zA-Z_]([a-zA-Z0-9_])*
			 ;
Time:Hour':'Minute
	;
fragment Hour:[0-9]
			 |'1'[0-9]
			 |'2'[0-3]
			 ;
fragment Minute:[0-5][0-9]
			   ;
/// XXX:<Rgb_color> must match as many as whitespaces.
Rgb_color:'('' '*Rgb' '*','' '*Rgb' '*','' '*Rgb' '*')'
		 ;
fragment Rgb:Rgb1
			|Rgb2
			|Rgb3
			;
fragment Rgb1:[0-9]
			 ;
fragment Rgb2:[1-9][0-9]
			 ;
fragment Rgb3:'1'[0-9][0-9]
			  |'2'[0-4][0-9]
			  |'25'[0-5]
			  ;
Non_negative_value:Non_negative_number'.'Non_negative_number
				  ;
Non_negative_number:[0-9]+
				   ;


// skipping tokens.
WS:[ \t\n\r]+->skip
  ;


// comments.
LINE_COMMENT:'//'(~[\n])*->skip
			;
BLOCK_COMMENT:'/*'.*?'*/'->skip
			 ;