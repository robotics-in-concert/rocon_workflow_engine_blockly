var _ = require('lodash')
  , Settings = require('./model').Settings;


module.exports = function(app, db){

  var _getItems = function(cb){
    var col = db.collection('settings');
    col.findOne({key: 'cento_authoring_items'}, function(e, data){
      var items = data ? data.value.data : [];
      cb(e, items);
    });

  };

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


  app.get('/api/param/:key', function(req, res){
    var k = req.params.key;
    Settings.findOne({key: k}, function(e, row){
      res.send(row.value);
    });
  });


  app.post('/api/engine/reset', function(req, res){

    global.restartEngine();
    // $engine.clear()
    res.send({result: true});


  });
  app.post('/api/engine/load', function(req, res){

    var itemIds = req.body.blocks;

    Settings.findOne({key: 'cento_authoring_items'}, function(e, row){
      var items = row.value.data;
      var items_to_load = _.filter(items, function(i){
        return _.contains(itemIds, i.id);
      });

      var titles = _.map(items_to_load, 'title');
      var pid = engineManager.startEngine();
      engineManager.run(pid, titles);

      res.send({result: true});
    });


  });
};
