var _ = require('lodash');


module.exports = function(app){


  app.get('/', function(req, res){
    res.render('engine');
  });
  app.get('/prezi', function(req, res){
    res.render('prezi');
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


  app.get('/ui/:name', function(req, res){
    var name = req.params.name;

    var meta = global.engineManager.ui_manager.ui[name];
    
    if(!meta){
      var el = '<div id="container" vertical layout>no ui found.</div>'
    }else{
      console.log(meta);


      var _build = function(meta){
        if(meta.type == 'vertical' || meta.type == 'horizontal'){

          if(meta.children){
            var childrenEl = _.map(meta.children, function(c){
              return _build(c);
            }).join("");
          }
          return '<div flex layout '+meta.type+'>'+childrenEl+'</div>';

        }else if(meta.type == 'button'){
          return '<rocon-button flex rocon_id="'+meta.name+'" text="'+meta.text+'"></rocon-button>';
        }else if(meta.type == 'text'){
          return '<rocon-text flex rocon_id="'+meta.name+'" text="'+meta.text+'"></rocon-text>';
        }


      };

      
      var el = '<div id="container" vertical layout>'+_build(meta[0])+'</div>'
    }
    console.log(el);

    // res.send(global.engineManager.ui_manager.ui);
    res.render('ui', {el:el});
  });
  app.post('/api/engine/reset', function(req, res){

    global.restartEngine();
    // $engine.clear()
    res.send({result: true});


  });
};
