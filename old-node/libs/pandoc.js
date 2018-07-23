
const punycode = require('punycode');
var fs = require('fs');
var path = require('path');
var childProcess = require('child_process');
var execSync = childProcess.execSync;

var w = execSync('which pandoc').toString();

if (typeof(w) !== 'string'){
  throw 'Is pandoc installed?';
}
var pandocPath = w.substr(0, w.length - 1);

exports.cleanAscii = function(file){
  // get markdown
  var text;
  try {
    text = fs.readFileSync(file, 'ascii');
    res = punycode.toASCII(text);
    fs.writeFileSync(file, res, {encoding: 'ascii'});
  }
  catch (err) {
    console.log(err);
    throw err;
  }
}

exports.convertToHTML = function(inFile) {

  var output_type;
  switch(path.extname(inFile)){
    case ".md":
      output_type = 'markdown';
      break;
    case ".rst":
      output_type = 'rst';
      break;
    default:
      throw "convertHTML: unknown file extention, must be md or rst";
  }

  // add --quiet to surpress warnings
  var cmd = pandocPath + ' -f ' + output_type + ' -t html5-smart ' + inFile;

  try {
    var ret = execSync(cmd).toString();
    // console.log(ret);
    return ret;
  }
  catch(err) {
    console.log('PANDOC ERROR: ', err);
    throw err;
  }
};

exports.writePDF = function(inFile, output_type) {

  var cmd = pandocPath + ' -f rst -V geometry:margin=1in -s -o ' + path.basename(input, path.extname(inFile)) + '.pdf ' + input;

  // console.log('DEBUG: ' + cmd);

  try {
    execSync(cmd).toString();
    // console.log(ret);
  }
  catch(err) {
    console.log('PANDOC ERROR: ', err);
    throw err;
  }
};
