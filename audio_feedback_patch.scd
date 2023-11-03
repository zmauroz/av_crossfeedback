( // Server Config
s.options.outDevice = "SC_OUT";
s.options.inDevice = "SC_IN";
s.options.blockSize = 32;
s.options.numOutputBusChannels = 16;
s.options.numInputBusChannels = 8;
s.options.sampleRate = 48000;
s.options.memSize = 65536*4;
// s.options.numWireBufs = 256;
s.reboot;
)

(
~g_buff.do{|i| i.free};
~g_buff = Array.fill(4, {Buffer.alloc(s, s.sampleRate * 5, 1)});
~g_buff.do{|i|
	i.fill(0, i.numFrames, 0);
};
)

(
var winenv = Env([0, 1, 0], [0.5, 0.5], [-8, 8]);
~env.free;
~env = Buffer.sendCollection(s, winenv.discretize, 1);
);

(
~audio_analysis.free;
~audio_analysis = Bus.control(s, 3);
);

NetAddr("192.168.0.52", 57110).sendMsg("/loudness", 5);

(
~snd_msg = { |port=57110|
	Routine{
		var val, loudness, centroid, flat;
		var n = NetAddr("192.168.0.52", port);
		loop{
			~audio_analysis.getn(3, {|a|
				loudness = a[0];
				centroid = a[1];
				flat = a[2];
			});
			n.sendMsg("/loudness", loudness);
			n.sendMsg("/centroid", centroid);
			n.sendMsg("/flat", flat);

			(1/30).wait;
	}}.play;
};
);
(
~reset_params = {
	fork{
		NetAddr("169.254.165.132", 57110).sendMsg("/sat_reset", 1);
		0.1.wait;
		NetAddr("169.254.165.132", 57110).sendMsg("/sat_reset", 0);
	}
}
)

(
Ndef(\fb1, {
	var ini = Impulse.ar(0);
	var fb, fqs=[0,0,0,0], comb, sat, res, rev=[0,0,0,0], grain=[0,0,0,0], allpass, loudness, centroid, flat, fb_low_sat;
	var fft;
	var n = NetAddr("169.254.165.132", 10000);

	fb = LocalIn.ar(4, ini);
	// AllPass
	allpass = AllpassC.ar(fb, 0.01, [\dly_a.kr(0.00927), \dly_b.kr(0.00785), \dly_c.kr(0.00482), \dly_d.kr(0.00937)], 0.001);

	// Reverb
	rev = LPF.ar(allpass, 18e3);
	rev =
		JPverb.ar(fb[0,1], 32, 0.371875, 1.94921875, 0.021875, 17.96875, 0.0, 0.34375, 0.3546875, 0.01, 1450.0822520097, 4450.794062356) ++
		JPverb.ar(fb[3,4], 32, 0.371875, 1.94921875, 0.021875, 17.96875, 0.0, 0.34375, 0.35546875, 0.01, 1450.0822520097, 4450.794062356);

	// FreqShift
	fqs = LPF.ar(fb, 18e3);
	fqs[0] = DelayC.ar((FreqShift.ar(fqs[0], 2.5)), 3, 2.4);
	fqs[1] = DelayC.ar((FreqShift.ar(fqs[1], 1.6)), 3, 1.2);
	fqs[2] = DelayC.ar((FreqShift.ar(fqs[2], -1.2)), 3, 2.07);
	fqs[3] = DelayC.ar((FreqShift.ar(fqs[3], -2.3)), 3, 2.7);
	fqs = HPF.ar(fqs.rotate, 50);

	// Saturation
	sat = LPF.ar(allpass, 300);
	sat = OSFold4.ar(sat*(\sat_a.kr(2) + \sat_var.kr(0).clip(0, 0.5)), -1, 1);
	sat = (LowShelf.ar(sat, 80, 1, 6) + HighShelf.ar(sat, 12e3, 1/2, 6) + sat)/3;
	sat = (LPF.ar(Compander.ar(sat, sat, 0.5, 1.0, 1/8, 0.01, 0.15), 150) + HPF.ar(sat, 150));

	// Granulation
	4.do{ |i|
		var trig = Dust.kr(\grain_density.kr(34.5));
		grain[i] = GrainBuf.ar(1, trig, TRand.ar(\g_min_d.kr(0.8), \g_max_d.kr(1.47), trig), ~g_buff[i], TChoose.kr(trig, [0.5, 2/3, 1, 0.25, 4/5]) * TChoose.kr(trig, [1, 1, 2, 1.5, 8/5, 2]) , TBrownRand.kr(0.0, 1.0, trig), 2, ~env);
	};
	grain = grain*0.25;
	grain = (grain * \grain_amp.kr(0.976));

	//Mix
	fqs = fqs * \fqs_amp.kr(0.626);
	sat = sat * \sat.kr(0.236);
	rev = rev.rotate * \rev_amp.kr(0.749);
	allpass = allpass * \allpass_amp.kr(0.7);

	fb = (fqs + sat + rev + allpass + (fb*(1)) + grain) * 0.3;

	fb = LPF.ar(LPF.ar(fb, 18e3), 16e3);
	fb = HPF.ar(HPF.ar(fb, 25), 25);


	fb = Compander.ar(fb, fb, 0.5, 1/2, 1, 0.005, 0.1);
	fb = Compander.ar(fb, fb, 0.707, 1, 1/4, 0.005, 0.1);

	LocalOut.ar(fb.rotate);

	fb = Limiter.ar(LeakDC.ar(fb), 0.8921);

	fft = FFT(LocalBuf(4096), fb.mean);
	loudness = Loudness.kr(fft);
	centroid = SpecCentroid.kr(fft); // Brightness -> spectrum's center mass
	flat = SpecFlatness.kr(fft); // 0 -> Sinewave || 1 -> Noise

	4.do{ |i|
		RecordBuf.ar(fb[i]*0.9, ~g_buff[i], 0, 1, 0.5, 1, 1);
	};

	Out.kr(~audio_analysis, [loudness, centroid, flat]);
	fb;
	// Out.ar(0, fb);
});

Ndef('fb1').set('dly_d', 0.0050912489648908, 'sat_var', 0.3, 'g_max_d', 1.36, 'fqs_amp', 0.488, 'allpass_amp', 0.685, 'dly_c', 0.0051168648601975, 'sat_a', 1.7, 'grain_density', 46.0, 'rev_amp', 0.855, 'g_min_d', 0.74, 'dly_a', 0.0078919889301062, 'sat', 0.145, 'grain_amp', 0.8, 'dly_b', 0.0052445480110124);
)

Ndef.gui;
NdefMixer(s);
Ndef.clear;

(
Spec.add(\dly_a, ControlSpec( 0.0001, 0.01, 'lin', 0, 0.00927, "seg") );
Spec.add(\dly_b, ControlSpec( 0.0001, 0.01, 'lin', 0, 0.00785, "seg") );
Spec.add(\dly_c, ControlSpec( 0.0001, 0.01, 'lin', 0, 0.00482, "seg") );
Spec.add(\dly_d, ControlSpec( 0.0001, 0.01, 'lin', 0, 0.00937, "seg") );
Spec.add(\sat, ControlSpec(0.0, 2.0, 4, 0.001, 0.236, 'amp'));
Spec.add(\fqs_amp, ControlSpec(0.0, 2.0, 4, 0.001, 0.7, 'amp'));
Spec.add(\grain_amp, ControlSpec(0.0, 2.0, 4, 0.001, 0.976, 'amp'));
Spec.add(\rev_amp, ControlSpec(0.0, 2.0, 4, 0.001, 0.626, 'amp'));
Spec.add(\allpass_amp, ControlSpec(0.0, 2.0, 4, 0.001, 0.7, 'amp'));
Spec.add(\grain_density, ControlSpec(1.0, 100.0, -4, 0.5, 34.5, 'dens'));
Spec.add(\g_min_d, ControlSpec(0.001, 1.5, -4, 0.01, 0.8, 'seg'));
Spec.add(\g_max_d, ControlSpec(0.001, 1.5, -4, 0.01, 1.47, 'seg'));
Spec.add(\sat_a, ControlSpec(0.1, 4, -4, 0.01, 1.47, 'seg'));
)


( //  OSC from TD
~d1.free;
~d1 = OSCFunc({ |val|
	var d1 = Lag3.kr(val[1], 0.001);
	d1 = Lag3.kr(d1.linlin(-1.0, 1.0, 0.0001, 0.01), 0.001);
	Ndef(\fb1).set(\dly_a, d1);
	// d1.postln;
}, 'd1');

~d2.free;
~d2 = OSCFunc({ |val|
	var d2 = Lag3.kr(val[1], 0.01);
	d2 = Lag3.kr(d2.linlin(-1.0, 1.0, 0.0001, 0.01), 0.001);
	Ndef(\fb1).set(\dly_b, d2);
	// d2.postln;
}, 'd2');

~d3.free;
~d3 = OSCFunc({ |val|
	var d3 = Lag3.kr(val[1], 0.01);
	d3 = Lag3.kr(d3.linlin(-1.0, 1.0, 0.0001, 0.01), 0.001);
	Ndef(\fb1).set(\dly_c, d3);
	// d1.postln;
}, 'd3');

~d4.free;
~d4 = OSCFunc({ |val|
	var d4 = Lag3.kr(val[1], 0.01);
	d4 = Lag3.kr(d4.linlin(-1.0, 1.0, 0.0001, 0.01), 0.001);
	Ndef(\fb1).set(\dly_d, d4);
	// d1.postln;
}, 'd4');

~st.free;
~st = OSCFunc({ |val|
	var aux = Lag3.kr(val[1], 0.01);
	aux = Lag3.kr(aux.linlin(0.0, 1.0, 0.7, 4.0), 0.1);
	Ndef(\fb1).set(\sat_a, aux);
	// aux.postln;
}, 'sat');

~ap.free;
~ap = OSCFunc({ |val|
	var aux = Lag3.kr(val[1], 0.01);
	aux = Lag3.kr(aux.linlin(0.0, 1.0, 0.3, 1.0), 0.1);
	Ndef(\fb1).set(\allpass_amp, aux);
	// aux.postln;
}, 'alp');
)

(
~d1.free; ~d2.free; ~d3.free; ~d4.free; ~st.free; ~ap.free;
)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Presets
Ndef('fb1').set('dly_d', 0.0087952380952381, 'sat_var', -5.8871741294861, 'g_max_d', 1.36, 'fqs_amp', 0.998, 'dly_c', 0.0090571428571429, 'allpass_amp', 0.749, 'sat_a', 3.0, 'grain_density', 46.0, 'rev_amp', 0.934, 'g_min_d', 0.74, 'dly_a', 0.0084809523809524, 'sat', 0.3, 'grain_amp', 0.874, 'dly_b', 0.0093190476190476);