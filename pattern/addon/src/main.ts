import { createRequire } from 'module'
const require = createRequire(import.meta.url)
const addon = require('../build/Release/module.node')

console.log(addon.getValue())