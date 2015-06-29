Polymer components
==================


* polymer components 의 경로 : public/polymer_components




## Polymer component 생성

1. public/polymer_components 하위에 원하는 `component이름.html` 생성 (예, video.html)

1. Grid 레이아웃을 위해 /css/rocon-component.css 를 포함
1. components 의 내용전체를 `<div id="rocon-component-wrapper"></div>` 로 감싸준다.

1. `<script></script>` 태그 안에는 원하는 내용의 스크립트 작업
1. `rocon-compoent-wrapper` div 안에는 원하는 마크업을 작성한다.


### 예제

```html
<link rel="import" href="../components/polymer/polymer.html">
<polymer-element name="rocon-text" attributes="text" >
  <template>
  <link rel="stylesheet" href="../css/rocon-component.css">
  <style>
    :host {
      background-color: #eee;
    }
  </style>
    <div id="rocon-component-wrapper">
      <div align-center>{{text}}</div>
    </div>
  </template>


  <script>
    Polymer('rocon-text', {
    });
  </script>

</polymer-element>
```


## Workflow Engine 과의 통신 처리 (pub/sub)

### Publish

Workflow Engine과에 pub 통신을 하는 부분은 rocon compoents 에서는 [core-signals](https://www.polymer-project.org/0.5/docs/elements/core-signals.html#core-signals) 로 추상화 되어 있다.


#### polymer components 에서 pub 하는 방법

* core-signals 의 fire 메서드를 이용한다.
	* name : 'rocon-component' 고정
	* data : publish 할 데이터 (JSON 으로 변경되어 std_msgs/String 타입으로 전달)

* 예제

```js
 Polymer('rocon-button', {
      handleTap: function(){
        this.fire('core-signal', {name: 'rocon-component', data: {'event': 'tap', id: this.rocon_id}});
        console.log('core signal fired');
      }
```


### Subscribe

workflow engine 에서 socket.io 의 `/ui` 채널로 전달되는 데이터로 처리한다.

* /ui 채널로 전달되는 데이터의 필드
	* rocon_id : polymer component 의 id 값
	* attr : 변경할 attribute (예, img 태그의 src)
	* value : 변경할 값.

