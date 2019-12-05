




~sample = Buffer.read(s,"C:/Users/Administrator/Downloads/looperman-l-2852656-0192348-jaybird-drums-80bpm.wav");

// Output signals
(
~mode = Bus.control;
{
	MapperOutput.kr(In.kr(~mode), 'Modeselektor', 0, 7);
}.play;


// Input signals

~freq = Bus.control;
{
	Out.kr(~freq, MapperInput.kr('Frequency', 20, 20000));
}.play;

~cutoff = Bus.control;
{
	Out.kr(~cutoff, MapperInput.kr('filter_cutoff', 100, 5000));
}.play;

~delayTime = Bus.control;
{
	Out.kr(~delayTime, MapperInput.kr('DelayTime', 0, 1));
}.play;


~trigger = Bus.control;
{
	Out.kr(~trigger, MapperInput.kr('Excitation', 0, 1));
}.play;

)

Ndef(\excPlay, {
	Out.kr(~trigger,LFPulse.kr(1));
}).play;

Ndef(\freqPlay, {
	Out.kr(~freq,SinOsc.kr(0.1,0.5,0.5));
}).play;

// Modeselektor

Ndef(\modePlay,{ arg mode = 0;
	Out.kr(~mode,mode)
}).play;


// Filter

Ndef(\filter,{
	var sig = BLowPass4.ar(LFSaw.ar(In.kr(~freq)),In.kr(~cutoff));
}).play;

// String - trigger mode

Ndef(\string,{
	var exc = K2A.ar(In.kr(~trigger));
	Pluck.ar(exc,exc,0.1,In.kr(~freq).linlin(0,1,0.001,0.01),3).dup;
}).play;


// Recordplayer velocity mode
Ndef().play
Ndef(\vinyl, {
	var buf, speed, env;
/*	env = EnvGen.kr(
		Env([0,1,0], [0.1, 0.1], \sin, 1),
		gate, doneAction: 20);*/
	speed = MouseX.kr(-2, 2);
	buf = PlayBuf.ar(1, ~sample, speed * BufRateScale.kr(~sample), loop: 1);
	buf = CombC.ar(buf,0.2,In.kr(~delayTime));
	Out.ar(0, (buf).dup );
}).play;

)