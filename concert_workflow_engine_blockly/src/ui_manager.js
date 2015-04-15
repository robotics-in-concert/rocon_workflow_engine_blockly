
var _ = require('lodash'),
  UUID = require('node-uuid'),
  EventEmitter2 = require('eventemitter2').EventEmitter2;



var UIManager = function(){
  this.ui = {};


};



UIManager.prototype.newUI = function(name, meta){
  this.ui[name] = meta;

  console.log('ui added', name, meta);

};



module.exports = UIManager;
