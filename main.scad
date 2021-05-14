function transform(v, q) = [v, q];
function vector(x=0, y=0, z=0) = [x, y, z];
function quaternion(w=1, x=0, y=0, z=0) = [w, x, y, z];

function vmagn(v) = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
function vnorm(v) = [v[0]/vmagn(v), v[1]/vmagn(v), v[2]/vmagn(v)];

function qmul(q0, q1) = [
    q0[0]*q1[0] - q0[1]*q1[1] - q0[2]*q1[2] - q0[3]*q1[3],
    q0[0]*q1[1] + q0[1]*q1[0] + q0[2]*q1[3] - q0[3]*q1[2],
    q0[0]*q1[2] - q0[1]*q1[3] + q0[2]*q1[0] + q0[3]*q1[1],
    q0[0]*q1[3] + q0[1]*q1[2] - q0[2]*q1[1] + q0[3]*q1[0]
];

function qrev(q) = [q[0], -q[1], -q[2], -q[3]];

function qrot(q, v) = qvec(qmul(
    qmul(q, quaternion(0, v[0], v[1], v[2])),
    qrev(q)
));

function qaxis(ang, v) = [
    cos(ang/2),
    vnorm(v)[0]*sin(ang/2),
    vnorm(v)[1]*sin(ang/2),
    vnorm(v)[2]*sin(ang/2)
];

function qvec(q) = [q[1], q[2], q[3]];

function qmagn(q) = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

function vdot(v0, v1) = v0[0]*v1[0] + v0[1]*v1[1] + v0[2]*v1[2];

function vcross(v0, v1) = [
    v0[1]*v1[2] - v0[2]*v1[1],
    v0[2]*v1[0] - v0[0]*v1[2],
    v0[0]*v1[1] - v0[1]*v1[0]
];

function angle_to(v0, v1, a) = atan2(vmagn(vcross(v0, v1)), vdot(v0, v1)) * (vdot(vcross(v0, v1), a) > 0 ? 1 : -1);

module rotation(q) {
    m = qmagn(q);
    ang = acos(q[0]/m);
    s = sin(ang)*sign(ang);
    rotate(ang*2, [s*q[1]/m, s*q[2]/m, s*q[3]/m]) children();
}

module axis(h=1) {
    color("red") rotate(90, [0, 1, 0]) cylinder(h=h, d1=0.2*h, d2=0);
    color("green") rotate(90, [-1, 0, 0]) cylinder(h=h, d1=0.2*h, d2=0);
    color("blue") cylinder(h=h, d1=0.2*h, d2=0);
}

rp = 8;
leg_h = 20;
leg_r = sqrt(2)/4 * leg_h;

angle_z = 0;
angle_y = 30 * cos($t * 360);
angle_x = ($t < 0.5) ? 30 * sin($t * 360) : 0;

targ_ori = qmul(
    qaxis(angle_z, [0, 0, 1]),
    qmul(
        qaxis(angle_y, [0, 1, 0]),
        qaxis(angle_x, [1, 0, 0])
    )
);

module platform() {
    union() {
        cylinder(h=0.5, r=rp, center=true, $fn=36);
        for (i = [0, 120, 240]) {
            rotate(i, [0, 0, 1]) rotate(90, [0, 1, 0]) cylinder(h=rp, d=1, $fn=8);
        }
    }
}

module ring(r) {
    difference() {
        cylinder(r=r+0.25, h=0.5, center=true, $fn=36);
        cylinder(r=r-0.25, h=1, center=true, $fn=36);
    }
}

ori0 = qrot(qmul(targ_ori, qaxis(-90, [0, 1, 0])), [0, 0, 1]);
alpha0 = acos(-sqrt(2) * ori0[2]);
mx0 = sin(alpha0);
my0 = -sqrt(2) * cos(alpha0) / 2;
q0 = angle_to([mx0, my0, 0], [ori0[0], ori0[1], 0], [0, 0, 1]);

ori1 = qrot(qmul(qmul(targ_ori, qaxis(120, [0, 0, 1])), qaxis(-90, [0, 1, 0])), [0, 0, 1]);
alpha1 = acos(-sqrt(2) * ori1[2]);
mx1 = sin(alpha1);
my1 = -sqrt(2) * cos(alpha1) / 2;
q1 = angle_to([mx1, my1, 0], [ori1[0], ori1[1], 0], [0, 0, 1]);

ori2 = qrot(qmul(qmul(targ_ori, qaxis(240, [0, 0, 1])), qaxis(-90, [0, 1, 0])), [0, 0, 1]);
alpha2 = acos(-sqrt(2) * ori2[2]);
mx2 = sin(alpha2);
my2 = -sqrt(2) * cos(alpha2) / 2;
q2 = angle_to([mx2, my2, 0], [ori2[0], ori2[1], 0], [0, 0, 1]);

module arm(q, alpha, col) {
    translate([0, 0, -leg_r]) rotate(q, [0, 0, 1]) {
        rotate(-90, [1, 0, 0]) translate([0, 0, 4]) rotate(30, [0, 0, 1]) cylinder(r=0.25, h=leg_r-4, $fn=3);
        translate([0, leg_r, 0]) rotate(45, [1, 0, 0]) {
            cylinder(h=1, r=0.25, $fn=6);
            color(col) rotate(alpha, [0, 0, 1]) rotate(-90, [1, 0, 0]) {
                rotate(30, [0, 0, 1]) cylinder(r=0.5, h=leg_h/2, $fn=3);
                translate([0, 0, leg_h/2]) rotate(90, [1, 0, 0]) {
                    rotate(30, [0, 0, 1]) cylinder(r=0.5, h=leg_h/2, $fn=3);
                    translate([0, 0, leg_h/2]) rotate(90, [1, 0, 0]) color("#0000ff") cylinder(h=2, r=0.5);
                }
            }
        }
    }
}

arm(q0, alpha0, "#ff8000");
arm(q1, alpha1, "#00ff80");
arm(q2, alpha2, "#8000ff");
translate([0, 0, -leg_r]) ring(leg_r-2);
rotation(targ_ori) {
    platform();
    axis(h=10);
}
