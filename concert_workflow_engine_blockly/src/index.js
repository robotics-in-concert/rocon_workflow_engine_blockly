var _ = require('lodash'),
  argv = require('minimist')(process.argv.slice(2)),
  colors = require('colors'),
  bodyParser = require('body-parser'),
  swig = require('swig'),
  http = require('http'),
  express = require('express'),
  socketio = require('socket.io'),
  socketio_wildcard = require('socketio-wildcard'),
  winston = require('winston'),
  EngineManager = require('./engine_manager'),
  Engine = require('./engine');


module.exports = function(){
  setupLogger();
  checkEnvVars();
  start();
}



function start(){

  var app = express(); 
  var server = http.createServer(app);
  var io = socketio(server);
  io.use(socketio_wildcard());
  io.of('/engine/client').use(socketio_wildcard());
  io.of('/blockly').use(socketio_wildcard());
  io.of('/prezi').use(socketio_wildcard());

  $io = io;

  io.on('connection', function(sock){
  });


  app.use(express.static('public'));
  app.use(bodyParser.json({limit: '50mb'}));

  // This is where all the magic happens!
  app.engine('html', swig.renderFile);

  app.set('view engine', 'html');
  app.set('views', __dirname + '/../views');

  app.set('view cache', false);
  swig.setDefaults({ cache: false });


  require('./routes')(app);

  server = server.listen(process.env.CONCERT_WORKFLOW_ENGINE_BLOCKLY_SERVER_PORT, function(){
    logger.info('Listening on port %d (%s)', server.address().port, process.env.NODE_ENV);
  });







  var engine_opts = _.defaults(argv.engine_options || {}, {
    publish_delay: +process.env.CONCERT_WORKFLOW_ENGINE_BLOCKLY_PUBLISH_DELAY,
    service_port: +process.env.CONCERT_WORKFLOW_ENGINE_BLOCKLY_SERVER_PORT
  });
  global.engineManager = new EngineManager(io, {engine_options: engine_opts});

  if(argv.workflow){
    argv.engine = true;
  }


  if(argv.engine){
    


    var workflows = argv.workflow;
    if(!_.isEmpty(workflows)){
      var pid = engineManager.startEngine();
      engineManager.run(pid, workflows);
    }




  }






};

function setupLogger(){
  winston.loggers.add('main', {
    console: {
      colorize: true,
      level: process.env.CONCERT_WORKFLOW_ENGINE_BLOCKLY_LOG_LEVEL,
      prettyPrint: true
    }

  });
  var logger = winston.loggers.get('main')
  // logger.cli()

  global.logger = logger;

  logger.debug('logger initialized');

};

function checkEnvVars(){

  ['CONCERT_WORKFLOW_ENGINE_BLOCKLY_SERVER_PORT',
    'CONCERT_WORKFLOW_ENGINE_BLOCKLY_ROSBRIDGE_URL',
    'CONCERT_WORKFLOW_ENGINE_BLOCKLY_PUBLISH_DELAY'].forEach(function(e){
      var v = process.env[e]
      if(v){
        logger.info(e, process.env[e].green);
      }else{
        logger.info(e, 'null'.red);
      }
    });

};

