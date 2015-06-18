
var _ = require('lodash'),
  Promise = require('bluebird'),
  EventEmitter2 = require('eventemitter2').EventEmitter2,
  ROSLIB = require('roslib'),
  Utils = require('./utils'),
  process_send2 = Utils.process_send2,
  util = require('util');

var Ros = function(opts){

  EventEmitter2.call(this, {wildcard: true, maxListeners:1024});
  var that = this;
  var options = this.options = opts;

  var ros_url = 'ws://' + options.rosbridge_address + ':' + options.rosbridge_port;
  var retry_op = Utils.retry(function(){
    logger.info('trying to connect to ros ' + ros_url);
    var connected = false;

    var ros = that.underlying = new ROSLIB.Ros({encoding: 'utf8'});

    ros.on('error', function(e){
      logger.info('ros error', e);
      if(!connected){
        retry_op.retry();
      }
    });
    ros.on('connection', function(){
      logger.info('ros connected');
      that.emit('status.started');

      that.waitForTopicsReady(['/concert/scheduler/requests']).then(function(){
        that.emit('status.ready');
      });
      connected = true;


    });
    ros.on('close', function(){
      logger.info('ros closed');
      // retry_op.retry();
    });
    ros.connect(ros_url);

  }, function(e){
    logger.error('ros connection failed', e);
    that.emit('status.start_failed');
    
  }, this.options.ros.retries, this.options.ros.retry_interval);


  this.subscribe_topics = [];
  this.publisher_list = {};
  this.publish_queue = [];
  this.publish_loop_timer = null;

  this.startPublishLoop();


};

util.inherits(Ros, EventEmitter2);


// public - promise version
Ros.prototype.waitForTopicsReady = function(required_topics){
  var that = this;
  var delay = this.options.action_delay || 2000;

  return new Promise(function(resolve, reject){
    var timer = setInterval(function(){
      that.underlying.getTopics(function(topics){
        var open_topics = _(topics).filter(function(t){ return _.contains(required_topics, t); }).value();
        logger.debug('topic count check : ', [open_topics.length, required_topics.length].join("/"), open_topics, required_topics);

        if(open_topics.length >= required_topics.length){
          clearInterval(timer);
          setTimeout(function(){ resolve(); }, delay);
        }
      });

    }, 1000);

  });


};


Ros.prototype.subscribe = function(topic, type, cb){

  var that = this;
  var listener = new ROSLIB.Topic({
    ros : this.underlying,
    name : topic,
    messageType : type
  });

  listener.subscribe(function(message) {
    logger.debug('Received message on ' + listener.name + ': ' + message);
    that.emit('subscribe.'+topic, message);
    if(_.isFunction(cb)){ cb.call(null, message); }
  });

  this.subscribe_topics.push({topic: topic, listener: listener});
};


Ros.prototype.startPublishLoop = function(){

  var that = this;
  that.publish_loop_timer = setInterval(function(){
    var data = that.publish_queue.shift();
    if(!data){
      return;
    }
      var topic = null;
      if (that.publisher_list.hasOwnProperty(data.topic) === true){
        topic = that.publisher_list[data.topic];
      }
      else{
        topic = new ROSLIB.Topic({
          ros : that.underlying,
          name : data.topic,
          messageType : data.type
        });
        that.publisher_list[data.topic] = topic;
      }

      var msg = new ROSLIB.Message(data.msg);
      setTimeout(function(){
        // And finally, publish.
        topic.publish(msg);
        logger.debug("[startPublishLoop]: " + topic.name);
        that.emit('publish', {name: data.name, type: data.type, payload: data.msg});
      }, that.options.publish_delay);

  }, that.options.publish_delay);

  logger.info('publish loop started');
};

Ros.prototype.stopPublishLoop = function(){
  clearInterval(this.publish_loop_timer);
  logger.info('publish loop stopped');
};


Ros.prototype.publish = function(topic, type, msg){
  this.publish_queue.push({topic: topic,  type: type, msg: msg});

};


Ros.prototype.getParam = function(key, callback){
  var that = this;
  return new Promise(function(resolve, reject){
    var param = new ROSLIB.Param({ros: that.underlying, name: key});
    param.get(resolve);
  });
};

Ros.prototype.unsubscribe = function(topic){
  var t = _.remove(this.subscribe_topics, {name: topic});
  t = t[0];
  t.listener.unsubscribe();
};
Ros.prototype.unsubscribeAll = function(){
  var engine = this;
  this.topics.forEach(function(t){
    t.listener.unsubscribe();
    logger.info("topic "+t.name+" unsubscribed");
  });
  this.subscribe_topics = [];
};

Ros.prototype.run_action = function(name, type, goal, onResult, onFeedback, onTimeout, options, isWaitForTopicsTimeout){
  if (isWaitForTopicsTimeout){
    onTimeout(goal);
    return;
  }

  var action_delay = this.options.action_delay || 2000;

  var options = _.defaults(options || {}, {
    timeout: -1
  });
  options.timeout = +options.timeout;

  logger.info("run action : " +  name + " " + type + " " + JSON.stringify(goal));

  var ac = new ROSLIB.ActionClient({
    ros : this.underlying,
    serverName : name,
    actionName : type
  });

  var param_goal = goal;
  var ros_goal = new ROSLIB.Goal({
    actionClient : ac,
    goalMessage : param_goal
  });


  var timeout_h = null;
  var timedout = false;

  var timer_goal_sender = "";
  var is_goal_sended = false;

  var _onResult = function(x){ if(!timedout){ clearTimeout(timeout_h);} onResult(x); };
  var _onFeedback = function(x){ if(!timedout){ onFeedback(x);} };
  var _onStatus = function(x){ if(!timedout){ is_goal_sended = true; }};

  ros_goal.on('feedback', _onFeedback);
  ros_goal.on('result', _onResult);
  ros_goal.on('status', _onStatus);

  timer_goal_sender = setInterval(function(){
    if(!is_goal_sended){
      logger.info("Sending goal processing untill receiving");
      ros_goal.send();
    }
    else{
      logger.info("Finish sending goal processing");
      clearInterval(timer_goal_sender);
      timer_goal_sender = "";
    }
  }, action_delay);

  if(options.timeout >= 0){
    timeout_h = setTimeout(function(){
      timedout = true;
      onTimeout(param_goal);
    }, options.timeout);
  }

};

module.exports = Ros;
