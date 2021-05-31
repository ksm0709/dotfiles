func! config#before() abort
   let g:mapleader  = '`'
   " after this line, when you using <leader> to defind key bindings
   " the leader is ,
   " for example:
   nnoremap <leader>w :w<cr>
   " this mapping means using `,w` to save current file.
   "
   "버퍼 새로열기
   nmap <leader>b :enew<CR>
   "다음 버퍼로
   nmap <leader>h :bprevious<CR>
   "이전 버퍼로
   nmap <leader>l :bn<CR>
   "현재 버퍼를 닫고 이전 버퍼로 이동
   nmap <leader>x :bp <BAR> bd #<CR>
   "모든 버퍼와 각 버퍼상태 출력
   nmap <leader>o :ls<CR>
   "현재 버퍼 강제 저장
   nmap <leader>s ::w!<CR>
endf
