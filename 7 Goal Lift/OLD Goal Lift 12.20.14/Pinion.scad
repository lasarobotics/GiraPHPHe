
//gear(pitch_radius = 1.125*25.4, teeth = 15, height = 10, pressure_angle = 30, clearance = .5, backlash = .25, helix_angle = 0);
rack(mating_pitch_radius=1.125*25.4, mating_teeth=15, teeth = 40, height = 10, base_depth = 20, helix_angle = 0, backlash = .25, clearance = .5, pressure_angle = 30);

/* GEAR */
module gear(
		pitch_radius,
		teeth,
		height,
		pressure_angle=28,
		clearance=0,
		backlash=0,
		involute_facets=5,
		helix_angle=0,
		helix_segments=1){
// Properties
base_radius=pitch_radius*cos(pressure_angle);
addendum = 2*pitch_radius/teeth;
circular_pitch = pitch_radius*6.28318531/teeth;
outside_radius = pitch_radius + addendum;
dedendum = addendum+clearance;
root_radius = pitch_radius-dedendum;
// Calculations
min_t=base_radius<root_radius?angle(root_radius,base_radius):0;
max_t=angle(outside_radius,base_radius);
delta_t=(max_t-min_t)/(involute_facets-1);
half_circular_thickness_angle = 28.64788976/pitch_radius*(circular_pitch-backlash)*.5;
pitch_involute_point=involute(angle(pitch_radius,base_radius),base_radius);
offset=atan2(pitch_involute_point[1],pitch_involute_point[0]);
// Construction
linear_extrude(height=height,twist=tan(-helix_angle)*height*57.29577951/pitch_radius,slices=helix_segments,convexity=teeth){
	circle(r=root_radius,$fn=teeth);
	for(i=[0:involute_facets-2]){
		assign(point1=rotate(involute(min_t+i*delta_t,base_radius),-offset-half_circular_thickness_angle),
		point2=rotate(involute(min_t+(i+1)*delta_t,base_radius),-offset-half_circular_thickness_angle),
		delta_theta=360/teeth){
			for(theta=[0:teeth-1]){
				rotate(theta*delta_theta)
					polygon(points=[[0,0],point1,point2,mirror(point2),mirror(point1)],paths=[[0,1,2,3,4]]);
			}
		}
	}
}}

/* RACK GEAR */
module rack(
		mating_pitch_radius,
		mating_teeth,
		teeth,
		height,
		base_depth,
		pressure_angle=28,
		clearance=0,
		backlash=0,
		helix_angle=0){
// Properties
addendum=2*mating_pitch_radius/mating_teeth;
dedendum=addendum+clearance;
offset_a=tan(pressure_angle)*addendum;
offset_d=tan(pressure_angle)*dedendum;
half_tooth_width=mating_pitch_radius*1.57079633/mating_teeth;
helix_offset=tan(helix_angle)*height;

// Construct teeth
union(){
for(i=[0:teeth-1]){
	translate([i*half_tooth_width*4-half_tooth_width,0,0])
		polyhedron(points=[[-offset_d+backlash/4,-dedendum,0],[offset_a+backlash/4,addendum,0],[2*half_tooth_width-offset_a-backlash/4,addendum,0],[2*half_tooth_width+offset_d-backlash/4,-dedendum,0],[-offset_d+backlash/4+helix_offset,-dedendum,height],[offset_a+backlash/4+helix_offset,addendum,height],[2*half_tooth_width-offset_a-backlash/4+helix_offset,addendum,height],[2*half_tooth_width+offset_d-backlash/4+helix_offset,-dedendum,height]],faces=[[0,1,2],[2,3,0],[4,5,6],[6,7,4],[0,1,4],[1,4,5],[1,2,5],[5,6,2],[2,3,6],[6,7,3],[0,3,4],[4,7,3]],convexity=1);
}}
// Construct base
if(base_depth<dedendum){echo("ERROR: Rack base_depth too small.");}
x=-offset_d-half_tooth_width+backlash/4;
y=-base_depth;
x2=x+helix_offset;
width=half_tooth_width*4*(teeth-1)+half_tooth_width*2+2*offset_d-backlash/2;
length=base_depth-dedendum;
union(){
polyhedron(points=[[x,y,0],[x,y+length,0],[x+width,y+length,0],[x+width,y,0],[x2,y,height],[x2,y+length,height],[x2+width,y+length,height],[x2+width,y,height]],faces=[[0,1,2],[2,3,0],[4,5,6],[6,7,4],[0,1,4],[1,4,5],[1,2,5],[5,6,2],[2,3,6],[6,7,3],[0,3,4],[4,7,3]]);
}}

/* FUNCTIONS */
function toRadians(angle) = angle*0.01745329;
function toDegrees(angle) = angle*57.29577951;
function rotate(point,angle)=[point[0]*cos(angle)-point[1]*sin(angle), point[1]*cos(angle)+point[0]*sin(angle)];
function mirror(point)=[point[0],-point[1]];
// [x,y] for 't' on parametric involute with base radius 'b'
	function involute(t,b)=b*[cos(t)+toRadians(t)*sin(t),sin(t)-toRadians(t)*cos(t)];
// 't' for parametric involute with base radius 'b' at radius 'r'
	function angle(r,b)=toDegrees(sqrt(pow(r/b,2)-1));
