open Batteries

let safe_hd = function
  | x::xs -> Some x
  | _ -> None

let safe_tl = function
  | x::xs -> Some xs
  | _ -> None

(* http://stackoverflow.com/questions/3989776/transpose-of-a-list-of-lists *)
let rec transpose_list list = match list with
  | []             -> []
  | []   :: xss    -> transpose_list xss
  | (x::xs) :: xss -> (x :: List.filter_map safe_hd xss) :: transpose_list (xs :: List.filter_map safe_tl xss)

let main () =
  let input = Std.input_lines Pervasives.stdin in
  let input = Enum.map String.to_list input in
  let input = List.of_enum input in
  let input = transpose_list input in
  let input = List.map (List.sort_unique compare) input in
    List.iter 
      (fun ch ->
	 match ch with
	   | [] -> assert false
	   | [x] -> Printf.printf "%c" x
	   | _::_::_ -> Printf.printf "?"
      )
      input;
    Printf.printf "\n"

let _ = main ()

