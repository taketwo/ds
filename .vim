let &makeprg="make\ -C".expand('<sfile>:p:h')."/build"
augroup AddSnippets
    autocmd!
    autocmd FileType cpp :UltiSnipsAddFiletypes pcl.eigen.cpp
augroup END
