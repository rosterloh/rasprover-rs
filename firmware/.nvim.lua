-- You must enable the exrc setting in neovim for this config file to be used.
local rust_analyzer = {
    cargo = {
        target = "xtensa-esp32-none-elf",
        allTargets = false,
    },
}
rust_analyzer.cargo.extraEnv = { RUST_TOOLCHAIN = "esp" }
rust_analyzer.check = { extraEnv = { RUST_TOOLCHAIN = "esp" } }
rust_analyzer.server = { extraEnv = { RUST_TOOLCHAIN = "stable" } }

-- Note the neovim name of the language server is rust_analyzer with an underscore.
vim.lsp.config("rust_analyzer", {
    settings = {
        ["rust-analyzer"] = rust_analyzer
    },
})
