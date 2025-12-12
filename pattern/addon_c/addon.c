#include <node_api.h>
#include <stdio.h>

// Add pure c/cpp code here.
void execute() {
  printf("Hello from C!\n");
}

/**
 * Node.js Wrapper Function to be callable from JavaScript.
 * 
 * Params:
 *   env  - Node.js environment handle (for memory management, value creation).
 *   info - Function call information (arguments, this context).
 * 
 * Returns:
 *   napi_value - JavaScript value (NULL = undefined).
 */
napi_value Run(napi_env env, napi_callback_info info) {
  execute();
  return NULL;
}

/**
 * Module Initialization.
 * Auto-executed when module is required().
 * Registers functions to module.exports.
 * 
 * Params:
 *   env  - Node.js environment handle.
 *   exports - module.exports object (add functions here).
 * 
 * Returns:
 *   napi_value - Modified exports object.
 */
napi_value Init(napi_env env, napi_value exports) {
  napi_status status; // Stores API call result (success/fail).
  napi_value fn;      // Will store the created JavaScript function.

  // Convert c/cpp function to JavaScript function.
  status = napi_create_function(env, NULL, 0, Run, NULL, &fn);
  if (status != napi_ok) return NULL;

  // Add function to exports object.
  status = napi_set_named_property(env, exports, "run", fn);
  if (status != napi_ok) return NULL;

  return exports;
}

// Module Registration Macro.
// Registers addon with Node.js (name from binding.gyp target_name).
// Calls Init() when module is loaded via require()
NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)

// npx node-gyp build