var _ = require('lodash'),
  args = require('minimist')(process.argv.slice(2)),
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


options = _.defaults(args, {
  port: 9999,
  rosbridge_port: 9091,
  rosbridge_address: 'localhost',
  log_level: 'info',
  publish_delay: 100,
  action_delay: 2000,
  ros: {
    retries: 0,
    retry_interval: 3000
  }
})


process.setMaxListeners(1024);




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

  server = server.listen(options.port, function(){
    logger.info('Listening on port %d (%s)', server.address().port, process.env.NODE_ENV);
  });

  global.engineManager = new EngineManager(io, options);

    


  // var workflows = argv.workflow;
  // if(!_.isEmpty(workflows)){
    // var pid = engineManager.startEngine();
    // engineManager.run(pid, workflows);
  // }










};

function setupLogger(){
  winston.loggers.add('main', {
    console: {
      colorize: true,
      level: options.log_level,
      prettyPrint: true
    }

  });
  var logger = winston.loggers.get('main')
  // logger.cli()

  global.logger = logger;

  logger.debug('logger initialized');

};

function checkEnvVars(){
  logger.info(args);
};

