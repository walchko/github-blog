
var fs = require('fs');
// var path = require('path');
var childProcess = require('child_process');
var execSync = childProcess.execSync;

var w = execSync('which jupyter').toString();

if (typeof(w) !== 'string'){
	throw 'Is jupyter installed?';
}
var jupyterPath = w.substr(0, w.length - 1);  // remove return

exports.convertToHTML = function(inFile){
	const cmd = jupyterPath + ' nbconvert --template basic --log-level=0 --stdout ' + inFile;

	try {
		var ret = execSync(cmd).toString();
		// console.log(ret);
		return ret;
	}
	catch(err) {
		console.log('JUPYTER ERROR: ', err);
		throw err;
	}
}

exports.zipFolder = function(file, folder, output){
	// file: zip file name
	// folder: path to folder to zip
	// output: where to move zip file to when done
	const cmd = 'zip -r ' + file +'.zip ' + folder;
	try {
		execSync(cmd);
		fs.renameSync(file + '.zip', output + '/' + file + '.zip');
	}
	catch(err) {
		console.log('JUPYTER ERROR: ', err);
		throw err;
	}
}
