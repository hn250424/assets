{
    "targets": [
        {
            "target_name": "module",
            "sources": ["src/native/main.cpp"],
            "include_dirs": [
                "<!(node -p \"require('node-addon-api').include\")",
                "<!(node -p \"require.resolve('node-addon-api') + '/../' \")"
            ],
            "dependencies": [
                "<!(node -p \"require('node-addon-api').gyp\")"
            ],
            "defines": ["NAPI_DISABLE_CPP_EXCEPTIONS"]
        }
    ]
}