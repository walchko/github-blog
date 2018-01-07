
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

// pandoc  -f rst --mathjax test.rst -t html5-smart -o index.html -s -c pygment.css
// exports.convertMarkdown = function(input, output_type, args, callback) {
//   // console.log('DEBUG: ' + input);
//   // cleanAscii(input);
//
//   var cmd;
//   switch (output_type) {
//     case 'html':
//       cmd = pandocPath + ' -f markdown -t html5 ' + input;
//     case 'pdf':
//       cmd = pandocPath + ' -V geometry:margin=1in -s -o ' + path.basename(input, '.md') + '.pdf ' + input;
//   }
//
//   // console.log('DEBUG: ' + cmd);
//
//   try {
//     var ret = execSync(cmd).toString();
//     // console.log(ret);
//     callback(null, ret);
//   }
//   catch(err) {
//     console.log('PANDOC ERROR: ', err);
//     callback(err, null);
//   }
// };

// pandoc  -f rst --mathjax test.rst -t html5-smart -o index.html -s -c pygment.css
// exports.convertMarkdown2 = function(input, output_type) {
//   // console.log('DEBUG: ' + input);
//   // cleanAscii(input);
//
//   // find out if args was passed to function
//   // if (typeof(args) === 'undefined'){
//   //   console.log('no optional args passed');
//   //   args = args || ' ';  // need better
//   // }
//
//   var cmd;
//   switch (output_type) {
//     case 'html':
//       cmd = pandocPath + ' -f markdown -t html5-smart ' + input;
//       break;
//     case 'pdf':
//       cmd = pandocPath + ' -f markdown -V geometry:margin=1in -s -o ' + path.basename(input, '.md') + '.pdf ' + input;
//       break;
//   }
//
//   // console.log('DEBUG: ' + cmd);
//
//   try {
//     var ret = execSync(cmd).toString();
//     // console.log(ret);
//     return ret;
//   }
//   catch(err) {
//     console.log('PANDOC ERROR: ', err);
//     throw err;
//   }
// };

// exports.convertRST = function(input, output_type) {
//   // console.log('DEBUG: ' + input);
//   // cleanAscii(input);
//
//   // find out if args was passed to function
//   // if (typeof(args) === 'undefined'){
//   //   console.log('no optional args passed');
//   //   args = args || ' ';  // need better
//   // }
//
//   var cmd;
//   switch (output_type) {
//     case 'html':
//       cmd = pandocPath + ' -f rst -t html5-smart ' + input;
//       break;
//     case 'pdf':
//       cmd = pandocPath + ' -f rst -V geometry:margin=1in -s -o ' + path.basename(input, '.rst') + '.pdf ' + input;
//       break;
//   }
//
//   // console.log('DEBUG: ' + cmd);
//
//   try {
//     var ret = execSync(cmd).toString();
//     // console.log(ret);
//     return ret;
//   }
//   catch(err) {
//     console.log('PANDOC ERROR: ', err);
//     throw err;
//   }
// };

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
