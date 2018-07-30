" ############## 기본 설정 #########################################################################

" 파일의 종류를 자동으로 인식 "
filetype on
filetype plugin on
filetype indent on

" 자동 문법 강조 "
syntax on 

set nocompatible         " Vim 디폴트 기능들을 사용함 "
set number                " 줄 번호를 붙임  "
set backspace=2         " 삽입 모드에서 백스페이스를 계속 허용 "
set autoindent             " 자동 들여쓰기 "
set cindent             " C 언어 자동 들여쓰기 "
set smartindent         " 역시 자동 들여쓰기 "
set nowrap                 " 자동 줄바꿈 안함  "
set nowrapscan             " 찾기에서 파일의 맨 끝에 이르면 계속하여 찾지 않음 "
set ignorecase             " 찾기에서 대/소문자를 구별하지 않음 "
set incsearch             " 점진적으로 찾기  "
set nobackup             " 백업파일을 만들지 않음 "
set nojoinspaces         " J 명령어로 줄을 붙일 때 마침표 뒤에 한칸만 띔 "
set ruler                 " 상태표시줄에 커서 위치를 보여줌 "
set tabstop=4             " 간격 "
set shiftwidth=4         " 자동 들여쓰기 간격 "
set keywordprg=ydic        " K를 눌렀을 때 실행할 명령어 "
set showcmd             " (부분적인)명령어를 상태라인에 보여줌 "
set showmatch             " 매치되는 괄호의 반대쪽을 보여줌 "
set autowrite             " :next나  :make 같은 명령를 입력하면 자동으로 저장 "
set linespace=3         " 줄간격 "
set title                 " 타이틀바에 현재 편집중인 파일을 표시 "
set statusline=\ %<%l:%v\ [%P]%=%a\ %h%m%r\ %F\     " 다중 문서 작업을 위한 Status 줄 "
set tags+=./tags        " add current directory's generated tags file to available tags "
set hidden 				" 버퍼를 수정한 직후 버퍼를 감춘다.
set grepprg=grep\-nH\ $*
"set shellslash

" ################ 플러그인 매니저 VUNDLE 관련 설정 ################################################

" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim
call vundle#begin()
	" alternatively, pass a path where Vundle should install plugins
	"call vundle#begin('~/some/path/here')

	" let Vundle manage Vundle, required
	Plugin 'gmarik/Vundle.vim'

	" The following are examples of different formats supported.
	" Keep Plugin commands between vundle#begin/end.
	" plugin on GitHub repo
	Plugin 'tpope/vim-fugitive'
	" plugin from http://vim-scripts.org/vim/scripts.html
	Plugin 'L9'
	" Git plugin not hosted on GitHub
	Plugin 'git://git.wincent.com/command-t.git'
	" git repos on your local machine (i.e. when working on your own plugin)
	"Plugin 'file:///home/gmarik/path/to/plugin'
	" The sparkup vim script is in a subdirectory of this repo called vim.
	" Pass the path to set the runtimepath properly.
	"Plugin 'rstacruz/sparkup', {'rtp': 'vim/'}
	" Avoid a name conflict with L9
	"Plugin 'user/L9', {'name': 'newL9'}
	
	Plugin 'The-NERD-Tree'
	Plugin 'AutoComplPop'
	Plugin 'snipMate'
	Plugin 'srcexpl'
	Plugin 'majutsushi/tagbar'
	Plugin 'vim-airline/vim-airline'
	Plugin 'vim-airline/vim-airline-themes'
	Plugin 'kien/ctrlp.vim'
	Plugin 'gerw/vim-latex-suite'	
	" All of your Plugins must be added before the following line
call vundle#end()            " required
	filetype plugin indent on    " required
	" To ignore plugin indent changes, instead use:
	"filetype plugin on
	"
	" Brief help
	" :PluginList       - lists configured plugins
	" :PluginInstall    - installs plugins; append `!` to update or just :PluginUpdate
	" :PluginSearch foo - searches for foo; append `!` to refresh local cache
	" :PluginClean      - confirms removal of unused plugins; append `!` to auto-approve removal
	"
	" see :h vundle for more details or wiki for FAQ
	" Put your non-Plugin stuff after this linespace
	" 
 set rtp+=/usr/local/lib/python2.7/dist-packages/powerline/bindings/vim/
 set t_Co=256
 set guifont=Liberation_Mono_for_Powerline:h10 
 let g:airline_powerline_fonts = 0
 let g:Powerline_symbols = 'unicode'
 " 버퍼 목록 켜기
 let g:airline#extensions#tabline#enabled = 1
"
" " 파일명만 출력
 let g:airline#extensions#tabline#fnamemod = ':t'
 let g:airline#extensions#left_sep = ''
 let g:airline#extensions#left_alt_sep = ''
 let g:airline_theme = 'bubblegum' 

 set laststatus=2

" #################### 단축키 설정  ######################################################################


" ---------- 함수 -----------------------------------------
" '< , '> : 비주얼 모드 블럭
" norm : 노말모드
func! CmtOn()    "주석 on
	exe "'<,'>norm i//"
endfunc

func! CmtOff()    "주석 off
	exe "'<,'>norm 2x"
endfunc

"---------- 정의, 설정 -----------------------------------Lock'

"latex
let g:tex_flavor='latex'
let g:tex_Leader=','


"ctags
"set tags=./tags
"set tags+=/opt/tags
"set tags+=/usr/tags
"cscope
"cs add /opt/cscope.out
"cs add /usr/cscope.out


let g:ctrlp_custom_ignore = {
  \ 'dir':  '\v[\/](\.(git|hg|svn)|\_site)$',
  \ 'file': '\v\.(exe|so|dll|class|png|jpg|jpeg)$',
  \}
let g:ctrlp_working_path_mode = 'r'

let mapleader = "`"

let g:SrcExpl_winHeight = 9
let g:SrcExpl_refreshTime = 100
let g:SrcExpl_jumpKey = "<ENTER>"
let g:SrcExpl_gobackKey = "<SPACE>"
let g:SrcExpl_isUpdateTags = 0

" // Use 'Exuberant Ctags' with '--sort=foldcase -R .' or '-L cscope.files' to
" " //  create/update a tags file
let g:SrcExpl_updateTagsCmd = "ctags --sort=foldcase -R ."
" // Set "<F12>" key for updating the tags file artificially
let g:SrcExpl_updateTagsKey = "<F12>"
" // Set "<F3>" key for displaying the previous definition in the jump list
let g:SrcExpl_prevDefKey = "<F9>"
" // Set "<F4>" key for displaying the next definition in the jump list
let g:SrcExpl_nextDefKey = "<F10>" 
let g:SrcExpl_pluginList = [
			\ "__Tag_List__",
			\ "_NERD_tree_"
			\] 

"---------- 키맵 ------------------------------------------
"버퍼 새로열기
nmap <leader>b :enew<CR>
"다음 버퍼로
nmap <leader>h :bprevious<CR>
"이전 버퍼로
nmap <leader>l :bn<CR>
"현재 버퍼를 닫고 이전 버퍼로 이동
nmap <leader>q :bp <BAR> bd #<CR>
"모든 버퍼와 각 버퍼상태 출력
nmap <leader>o :ls<CR>
"현재 버퍼 강제 저장
nmap <leader>s ::w!<CR>

"CtrlP Plugin관련 설정
nmap <leader>t :enew<CR>:term<CR>
nmap <leader>i :startinsert<CR>
nmap <leader>p :CtrlP<CR>
nmap <leader>pb :CtrlPBuffer<CR>
nmap <leader>pm :CtrlPMixed<CR>
nmap <leader>ps :CtrlPMRU<CR>

"입력모드 빠져나가기
imap jj <ESC>
"현재 커서에서 새로운줄 삽입
map <C-J> i<CR><ESC>
"비주얼모드에서 주석 처리
vmap <C-c> <esc>:call CmtOn() <cr>
"비주얼모드에서 주석 해제
vmap <C-x> <esc>:call CmtOff() <cr>
"노말모드에서 주석 처리
nmap <C-c> <esc>v:call CmtOn() <cr>
"노말모드에서 주석 해제
nmap <C-x> <esc>v:call CmtOff() <cr>

nmap cm ::!catkin_make -C catkin_ws<cr>

nmap <F1> :!bash<CR>
nmap <F2> [{v]}zf
nmap <F3> zo
nmap <F5> :NERDTreeToggle<CR>B
nmap <F6> :TagbarToggle<CR>
nmap <F7> :SrcExplToggle<CR>

"tnoremap <Esc> <C-\><C-n>
"tnoremap <A-h> <C-\><C-n><C-w>h
"tnoremap <A-j> <C-\><C-n><C-w>j
"tnoremap <A-k> <C-\><C-n><C-w>k
"tnoremap <A-l> <C-\><C-n><C-w>l
nnoremap <A-h> <C-w>h
nnoremap <A-j> <C-w>j
nnoremap <A-k> <C-w>k
nnoremap <A-l> <C-w>l

