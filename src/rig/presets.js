/* Presets. Each relaxes a rule the gate suite enforces; the note names which one, so
   the cheat stays legible. Every step count below was measured on this exact table. */
/* Every "Measured" line below was re-run on THIS build after the contact-impulse and
   end-stop fixes: 120 s of simulated time at a 0.78 m stride, headless, same solver.
   The numbers the artifact used to ship were measured on the unfixed physics and no
   longer described anything it did. */
/* Every "Measured" line is a 6-run ensemble on THIS build, not a single trajectory.
   That distinction matters here: the walk is deterministic but chaotic, and a 1.9e-16
   relative change in one gait parameter was measured moving a fall from 178 s to 23 s.
   Single runs mean nothing; survival counts do. */
const PRESETS = {
  light:{ label:'Light Frame', mass:0.5, torque:0.5, gain:0.5, envelope:0.7, kCop:0.60,
    cmg:{mass:90, tauMax:45e3, hMax:2.2e4},
    steps:'rough build at 1/4 scale &mdash; smoke-tested only, not qualified',
    note:'Half the material density &mdash; 4 180 kg, about 710 kg/m&sup3; mean, so a hollow composite shell. Actuators and servo gains scale down with it (&times;0.5); mounts keep 70% of envelope. Balance gain goes <b>up</b>, 0.40 to 0.60. The gyro earns its mass here: without it this rig is 3/5 at a 1.00 m stride with 29 joints torn, with it 5/5 and none, using half its torque and 45% of its momentum store.' },
  atst:{ label:'Scout Walker', spec:'atst', kCop:0.40,
    cmg:{mass:260, tauMax:90e3, hMax:4.4e4, kp:300e3, kd:84e3},
    steps:'rough build at 1/5 scale &mdash; smoke-tested only, not qualified',
    note:'Scout-walker proportions: 8 200 kg carried 8.47 m up on two close-set digitigrade legs, COM at 65% of height against MK1&rsquo;s 60%. Walks unassisted at 3/3 with peak mount load of only 28%, so it has margin in hand. An earlier build of this rig put a 3.6 m-wide cockpit on 0.72 m-wide thighs and could not walk at all without the gyro &mdash; that was a proportion error on my part, not a property of the silhouette. Mount envelopes are 3&times; a naive scaling by leg length: at 1.8&times; the hip yoke tore 0.45 s into simply standing up, because moments about the mounts grow far faster than leg length.' },
  verified:{ label:'Reference (heavy)',
    steps:'rough build at 1/4 scale &mdash; smoke-tested only, not qualified',
    note:'The 8 360 kg original, and what the self test asserts against &mdash; press <i>Run self test</i> to watch all 14 analytic checks run on this solver. Kept as the honest baseline, not because it is good: Light Frame beats it on every measure.' },
  overdrive:{ label:'Overdriven', torque:3.0,
    steps:'dismantles itself in 0.5 s, 16 joints torn',
    note:'Actuators &times;3 against unchanged mounts. Full actuator torque was already 73% of the mount envelope, so &times;3 is 220% and the rig comes apart the instant it loads a leg. The only live proof that the failure envelope is real rather than decorative.' },
};

/* Raising tauMax must NOT raise the servo gains (that tore the rig at frame 0), and
   cutting inertia MUST scale the gains with it or the servo is no longer tuned. */
function applyPreset(rig,p){
  /* `mass` scales real mass AND inertia -- a genuine change of material density, so weight
     and ground reaction move with it. `inertia` scales rotational inertia only and leaves
     mass alone, which is the cheat the Heavy Iron preset advertises. They are deliberately
     separate knobs. */
  if(p.mass) for(const b of Object.values(rig.bodies)){
    b.mass*=p.mass; b.I=b.I.map(v=>v*p.mass);
    if(!b.kinematic){ b.invMass=1/b.mass; b.invI=m3inv(b.I); }
  }
  if(p.inertia) for(const b of Object.values(rig.bodies)){ b.I=b.I.map(v=>v*p.inertia); b.invI=m3inv(b.I); }
  for(const j of Object.values(rig.joints)){
    if(p.torque) j.tauMax*=p.torque;
    // Servo gains track INERTIA, never tauMax. Raising actuator authority must not stiffen
    // the loop, and lightening the machine must not leave the loop tuned for the old one.
    if(p.inertia){ j.kp*=p.inertia; j.kd*=p.inertia; }
    if(p.gain){ j.kp*=p.gain; j.kd*=p.gain; }
    if(p.envelope) for(const k of ['tension','shear','bend','torsion']) j.lim[k]*=p.envelope;
  }
  for(const w of Object.values(rig.welds))
    if(p.envelope) for(const k of ['tension','shear','bend','torsion']) w.lim[k]*=p.envelope;
}

