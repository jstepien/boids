var Module = { boids_state: [] };
Module['noInitialRun'] = "No, thank you."
print = function(output) { Module.boids_state = eval(output); };
function start() {
	var args = ['64'];
	$(Module.boids_state).each(function(i, boid) {
		$(boid).each(function(j, val) {
			args.push('' + val);
		});
	});
	Module.arguments = args;
	Module.run();
	var context = $('canvas')[0].getContext('2d');
	context.fillStyle = "rgba(255, 255, 255, 0.08)";
	context.fillRect(0, 0, 640, 480);
	context.fillStyle = "rgb(0,0,0)";
	$(Module.boids_state).each(function(i, boid) {
		context.fillRect(boid[0], boid[1], 0.5, 0.5);
	});
	setTimeout(start, 10);
};
