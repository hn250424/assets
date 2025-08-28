declare module '*/build/Release/module.node' {
    interface AddonModule {
        getValue(): number
    }
    const addon: AddonModule
    export default addon
}