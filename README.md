# rovi_industrial


### 要件  
- (Mandatory))ロボットからのリクエスト  
 1. リセット  
 2. キャプチャ  
 3. ソルブ  
 4. レシピ選択  
 5. ファンクション選択  
- (Option)ロボットからのストリーミング(またはRoVIからのポーリング)  
 1. メカニカルインタフェース座標  
 2. ジョイント座標  

- Configuration  
以下パラメータによる

|name|type|description|
|:----|:----|:----|
|protocol|{fanuc,melfa...}|通信プロトコル選択|
|port|Int|ソケットサーバのポート番号|
|source_frame_id|String|マスターデータのフレームID|
|target_frame_id|String|Solverによって算出されたフレームID|


### Topics(to subscribe)  
|name|type|description|
|:----|:----|:----|
|response/cleare|Bool|リセット要求に対する応答|
|response/capture|Bool|キャプチャ要求に対する応答|
|response/solve|Bool|ソルブ要求に対する応答|
|response/recipe_load|Bool|レシピ選択要求に対する応答|
|response/function_load|Bool|ファンクション選択要求に対する応答|

### Topics(to subscribe)  
|name|type|description|
|:----|:----|:----|
|request/clear|Bool|リセット要求|
|request/capture|Bool|キャプチャ要求|
|request/solve|Bool|ソルブ要求|
|request/recipe_load|String|レシピ選択要求|
|request/function_load|String|ファンクション選択要求|

