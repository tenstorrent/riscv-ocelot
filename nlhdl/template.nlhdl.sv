/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

/*
<one or two lines: what this module is. Not how it works — that is the body
below. Keep it short; a paragraph here becomes a paragraph in the RTL.>
*/

// Comments in this file are copied verbatim into the generated RTL, so they are
// rationed: `//@req-<id>` tags (required, above the description each answers)
// and the short file description above. Everything else — behavior, rationale,
// context — is body prose, not a comment.

// Required, specify the module input/output and functionality
<|begin_module|>
  <|begin_parameters|>
    

  <|end_parameters|>

  <|begin_ports|>


  <|end_ports|>

  <|begin_logic|>


  <|end_logic|>

<|end_module|>

// Optional, this section specifies the intended performance of the module
<|begin_perf|>

<|end_perf|>

// Recommended, this section specifies which modules this module is dependent on
<|begin_dependencies|>

<|end_dependencies|>

// Required only for `mode: edit_existing` in hierarchy.yaml — i.e. when the
// sections above describe a DELTA against pre-existing RTL rather than a whole
// module. Bounds what the implementor may change. Delete this whole section for
// a new module (`mode: new`) or a regeneration (`mode: edit_generated`).
<|begin_edit_scope|>
  Target:           <file and module being edited>
  In scope:         <signals / blocks / ports the change may touch>
  Must not regress: <behavior that stays bit- and cycle-identical>
  Interface delta:  <new or widened ports/parameters only>
<|end_edit_scope|>

