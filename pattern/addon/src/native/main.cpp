#include <napi.h>

Napi::Number Return88(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    return Napi::Number::New(env, 88);
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
    exports.Set(Napi::String::New(env, "getValue"), Napi::Function::New(env, Return88));
    return exports;
}

NODE_API_MODULE(module, Init)
