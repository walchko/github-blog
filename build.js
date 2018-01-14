#!/usr/bin/env node

var pandoc = require('./pandoc.js')
var fs = require('fs')
var path = require('path')
var ejs = require('ejs');
// var program = require('commander');

const args = process.argv;
// console.log(args);

var BASE_PATH;

// push to webserver
if(args.length == 3){
  if(args[2] === 'surge'){
    console.log('Deploying to surge');
    BASE_PATH = 'https://walchko.surge.sh';
  }
  else if (args[2] === 'github'){
    console.log('Deploying to github');
    BASE_PATH = 'https://walchko.github.io';
  }
}
// just local testing
else {
  console.log('Local deployment');
  BASE_PATH = __dirname + '/html';
}


// function readAsciiFile(file){
//   // value of this?
//   // get markdown
//   var text;
//   try {
//     text = fs.readFileSync(mdFile, 'ascii');
//     return text;
//   }
//   catch (err) {
//     console.log(err);
//     throw err;
//   }
// }

/*
convertDoc
- inputfilename
- outputfilepath
- input file extention
- html template - ??? convertToHtml()

convertToPDF()?

pandoc.convertMarkdown(file, html | pdf) -> data
pandoc.convertRST(file, html | pdf) -> data
*/
function convertToHtml(inFile, template, outFile) {
  /*
  * Generic conversion function
  * - inFile: markdown or rst text filename
  * - template
  * - outFile: output file path
  */

  var result = pandoc.convertToHTML(inFile);
  var ext = path.extname(inFile);
  var htmlFile = outFile + '/' + path.basename(inFile, ext) + '.html';
  console.log('Wrote: ' + htmlFile);
  var html = template(
    {
      TOC: false,
      info: result,
      // path: __dirname + '/html'
      // path: 'html'
      path: BASE_PATH
    }
  );
  fs.writeFileSync(htmlFile, html);

}

function convertToPDF(inFile, outFile) {
  /*
   * Generic conversion function
   * - inFile: markdown or rst text filename
   * - outFile: output file path
   */

   // read the file and get the raw text
    var result;
    switch(path.extname(inFile)){
      case ".md":
        result = pandoc.convertMarkdown2(inFile, 'pdf');
        break;
      case ".rst":
        result = pandoc.convertRST(inFile, 'pdf');
        break;
    }
    var ext = path.extname(inFile);
    var pdfFile = outFile + '/' + path.basename(inFile, ext) + '.pdf';
    console.log('Wrote: ' + pdfFile);
    fs.writeFileSync(pdfFile, result);

}

function recursiveBuild(directory, template, output){
  console.log('-------------------------------------------')
  console.log('Searching: ' + directory);

  // get files
  var files;
  try {
    files = fs.readdirSync(directory);
  }
  catch (err) {
    console.log(err);
  }

  for (var i = 0; i < files.length; i++) {
      var currentFile = directory + '/' + files[i];

      switch (path.extname(files[i])) {
          case '.rst':
          case '.md':
              convertToHtml(currentFile, template, output);
              break;
          case '':
              if (fs.statSync(currentFile).isDirectory() == true){
                fs.mkdirSync(output + '/' + files[i]);
                recursiveBuild(currentFile, template, output + '/' + files[i]);
              }

              break;
          default:
              // copy to output directory
              var cp = output + '/' + files[i];
              fs.copyFileSync(currentFile, cp);
              console.log('Copied: ' + cp);
              break;
      }
  }
}

function buildTOC(source, template){
  const dirs = p => fs.readdirSync(p).filter(f => fs.statSync(path.join(p, f)).isDirectory());
  const folders = dirs(source);
  console.log(folders);
  console.log('-------------------------------------------')
  console.log('Searching for Topics: ');

  var toc = [];

  for (var i in folders){
    var folder = folders[i]
    console.log('folder: ' + folder);
    // toc[folder] = [];
    var sfolder = [folder];
    var sfiles = [];
    var files;

    try {
      files = fs.readdirSync(source + '/' + folder);
    }
    catch (err) {
      console.log(err);
    }
    for (var j in files){
      var f = files[j];
      // console.log(f);
      switch (path.extname(f)){
        case '.rst':
          f = path.basename(f, '.rst');
        case '.md':
          f = path.basename(f, '.md');
          console.log('Found: ' + f);
          // toc[folder].push({'name': f, 'path': source + '/' + folder + '/' + f});
          sfiles.push({'name': f.replace('_', ' ').replace('-', ' '), 'path': 'blog/' + folder + '/' + f + '.html'});
          break;
      }
    }
    sfolder.push(sfiles);
    toc.push(sfolder);

    // console.log(toc);
    // var html = template({TOC: toc, info: false, path: __dirname + '/html'});
    var html = template({TOC: toc, info: false, path: BASE_PATH});
    fs.writeFileSync('html/topics.html', html);
  }
}

function build(templateFile, directory, output){
  // delete old build and recreate it so we always have a clean build
  var execSync = require('child_process').execSync;
  execSync('rm -rf ' + output);

  fs.mkdirSync(output);

  // get template
  var ejs_string = fs.readFileSync(templateFile, 'utf8');
  var template = ejs.compile(ejs_string,{filename: __dirname + '/' + templateFile});

  // build topic page
  buildTOC(directory + '/blog', template);

  // search through source and build html
  recursiveBuild(directory, template, output);

}

build('template.ejs', 'source', 'html');
