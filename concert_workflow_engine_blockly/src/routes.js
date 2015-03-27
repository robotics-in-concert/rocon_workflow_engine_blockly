var _ = require('lodash');


module.exports = function(app){


  app.get('/', function(req, res){
    res.render('engine');
  });

  app.get('/ping', function(req, res){
    res.send('pong')
  });

  app.post('/api/engine/start', function(req, res){

    var payload = req.body;

    var pid = engineManager.startEngine();
    if(payload && payload.items && payload.items.length){
      engineManager.run(pid, payload.items);
    }
    res.send({status: 'ok', pid: pid});
  });

  app.post('/api/engine/stop', function(req, res){

    var payload = req.body;
    engineManager.killEngine(payload.pid);
    res.send({status: 'ok'});
  });


  app.post('/api/engine/reset', function(req, res){

    global.restartEngine();
    // $engine.clear()
    res.send({result: true});


  });
};
