###### Remove-Item Alias:curl  * curl.exe -> curl  

curl   
-X POST http://example.com/api/endpoint     * Method  
-H "Content-Type: application/json"         * Header  
-d '{"name": "John", "age": 30}'            * Request body  
-i | ConvertFrom-Json | Format-List         * Print with response header  

