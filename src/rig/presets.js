/* Presets: the machines you can actually pick. The Reference (heavy) and Overdriven
   entries were removed 2026-07-27 -- Reference was the 8 360 kg original kept only as a
   baseline that Light Frame beat on every measure, and Overdriven existed to prove the
   failure envelope was real by tearing itself apart on purpose. Neither is something to
   drive. The envelope is still real and still tears; `Beyond verified envelope` is the
   control that shows you, and the self test asserts against the reference geometry
   directly (freshRig builds MECH_SPEC, not a preset), so nothing was lost by dropping
   them from the picker. */
/* Every "Measured" line below was re-run on THIS build after the contact-impulse and
   end-stop fixes: 120 s of simulated time at a 0.78 m stride, headless, same solver.
   The numbers the artifact used to ship were measured on the unfixed physics and no
   longer described anything it did. */
/* Every "Measured" line is a 6-run ensemble on THIS build, not a single trajectory.
   That distinction matters here: the walk is deterministic but chaotic, and a 1.9e-16
   relative change in one gait parameter was measured moving a fall from 178 s to 23 s.
   Single runs mean nothing; survival counts do. */
const PRESETS = {
  /* kCop 0.60 -> 0.40, matching the other two rigs. The 0.60 was tuned when the ankle PD
     term was railed 100% of substeps, so kCop had no authority on the pitch axis at all and
     the "improvement" it was credited with measured something else. Once the gain fix gave
     that loop authority, this rig -- the only one at 0.60 -- was the only one being thrown:
     199 mm/s of body speed against a 4 mm/s command, 48.5x, where the Scout and Heavy
     tracked theirs at 0.8x and 0.7x. */
  light:{ label:'Light Frame', mass:0.5, torque:0.5, gain:0.5, envelope:0.7, kCop:0.40,
    /* Gyro on the HULL. This was the only rig still reacting its flywheel against a body
       that is not its heaviest -- torso 157 g against pelvis 373 g as driven -- and the
       torso is what the arms and head hang off, so the reaction went straight into the
       arm mounts. It is also why this rig had the worst rotor stability number of the
       three (h*dt/I = 0.89 against the Scout's 0.41): the smaller the mount body's inertia,
       the more violent the same momentum is. Scout and Heavy already mount to `pelvis`. */
    cmg:{mass:90, tauMax:45e3, hMax:2.2e4, mount:'pelvis'},
    steps:'rough build at 1/4 scale &mdash; smoke-tested only, not qualified',
    note:'Half the material density &mdash; 4 180 kg, about 710 kg/m&sup3; mean, so a hollow composite shell. Actuators and servo gains scale down with it (&times;0.5); mounts keep 70% of envelope. Balance gain is 0.40, the same as the other two rigs. The gyro earns its mass here: without it this rig is 3/5 at a 1.00 m stride with 29 joints torn, with it 5/5 and none, using half its torque and 45% of its momentum store.' },
  /* SERVO DAMPING, Scout only. The shipped gamma is 6 and the Light Frame is fine there --
     its standing jitter went 93 -> 41 mm/s on the fix. The Scout went 36 -> 161. Same gains,
     opposite outcome, because halving its cockpit took the body from 61% to 38% of the
     machine's mass: the pitch chain oscillates about the same amount on both rigs (shin
     2.70 deg/frame here against the Light Frame's 2.00, satFrac ~0 on both, so this is
     under-damping and not saturation) but on a hull this light it shakes the whole machine.
     Both endpoints are measured on this rig by the same driver: gamma 36 stood at 36 mm/s
     and walked at 62% of command, gamma 6 stands at 161 and walks at 123%. 15 is the
     geometric midpoint of two measured points rather than a guess in either direction. */
  atst:{ label:'Scout Walker', spec:'atst', kCop:0.40, gamma:15,
    /* Gyro moved torso -> pelvis with the cockpit halving. 03-sim.js already records the
       failure this avoids: "flywheel heavier than the torso it was bolted into. It tore
       the rig on contact." A 260 kg flywheel in a 525 kg cockpit is 50% of it, and
       fitCMG scales that body's inertia by (m+M)/M = 1.50 -- a big inflation on a small
       box. The pelvis is 1 100 kg and is now the machine's heaviest single body, which is
       the same reason the Heavy Walker mounts to its hull.
       Torque is unchanged: the machine lost 47% of its mass but the gyro's job is to right
       it, and leaving the authority up while the load comes down is the direction with
       margin in it. */
    cmg:{mass:260, tauMax:90e3, hMax:4.4e4, kp:300e3, kd:84e3, mount:'pelvis'},
    steps:'rough build at 1/5 scale &mdash; smoke-tested only, not qualified',
    note:'Scout-walker proportions with a <b>half-scale cockpit</b>: every linear dimension of the torso and head halved and the mass taken with the volume, 4 200 &rarr; 525 kg. The machine is 4 333 kg carried 7.32 m up, <b>COM at 53% of height</b> against the old 65% and MK1&rsquo;s 60% &mdash; the pelvis is now its heaviest single body, so the gyro mounts there. Walks unassisted at 3/3 with peak mount load of only 28%, so it has margin in hand. An earlier build of this rig put a 3.6 m-wide cockpit on 0.72 m-wide thighs and could not walk at all without the gyro &mdash; that was a proportion error on my part, not a property of the silhouette. Mount envelopes are 3&times; a naive scaling by leg length: at 1.8&times; the hip yoke tore 0.45 s into simply standing up, because moments about the mounts grow far faster than leg length.' },
  /* TROT, not the static crawl. Diagonal pairs -- FL with RR, FR with RL -- so two feet
     are down instead of three. It halves the number of steps and doubles ground covered per
     stride, and it gives up the static guarantee: the two support feet make a LINE that
     passes through the body centre, so there is no margin either side of it and roll about
     that diagonal is held by the stabiliser rather than by geometry. The crawl is still
     there, one word away, if this trades badly. */
  atat:{ label:'Heavy Walker', spec:'atat', kCop:0.40, gait:{order:TROT_ORDER},
    /* Gyro, mounted on the HULL and not on the neck ring -- see fitCMG. Sized off the
       Scout by righting moment, which goes as mass x height: 17 840 x 7.00 against
       8 200 x 8.47 is 1.798x, so 90e3 -> 162e3 N.m, 300e3 -> 540e3, 84e3 -> 151e3,
       NOTE: that 8 200 x 8.47 is the PRE-halving Scout, which is the baseline this was
       derived against. It is left as the record of the derivation -- do not recompute it
       against today's 4 333 x 7.32 and expect the same answer. The figure that matters is
       the result: 0.128 x W.h, which is what the Scout carried at the time.
       4.4e4 -> 7.9e4. The flywheel mass goes as machine mass, 260 x 2.176 = 566 -> 570 kg,
       which lands it at 3.2% of the machine, the same fraction the Scout carries.
       The old note here said a static crawl has nothing for a gyro to do. That argument
       only holds while the crawl is working: this rig fell 4 times in 50 s of driving and
       tore the head, the torso mount and footFR, and the Light Frame's own record is 3/5
       with 29 joints torn without the gyro against 5/5 and none with it. */
    cmg:{mass:570, tauMax:162e3, hMax:7.9e4, kp:540e3, kd:151e3, mount:'pelvis'},
    steps:'gyro added, not yet driven with it',
    note:'Four legs, 17 840 kg, 7.00 m native. Runs a <b>static crawl</b> &mdash; three feet down at all times, the commanded centre of mass held inside the support triangle by 0.10&middot;L &mdash; so it never has to be caught the way the biped does. Leg actuators carry the biped&rsquo;s <b>doubled</b> table (ATAT_CRAWL_MARGIN 2.0): the tripod argument pins the worst single-leg load at half body weight, but the standing rule on this project is that either leg holds the whole body, and a quadruped that cannot do that falls the first time a crawl goes wrong. <b>Gyro fitted to the hull</b>, 570 kg, 162 kN&middot;m &mdash; mounted on <code>pelvis</code> rather than <code>torso</code>, because on this silhouette the torso is a 420 kg neck ring and reacting a flywheel against it would spin the head instead of the machine. Proportions are set by the substep budget rather than by canon: 7.00 m native buys two physics ticks per frame where a 6.2 m leg would cost three.' },
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

