# Main software repository for Duckietown

This is the main software repository for [the Duckietown project][main]. It contains all software, as well as hardware projects.

- For an introduction to the project, see the site [duckietown.org][main].
- For extensive technical documentation, please refer to [the Duckiebooks][duckiebook].

[main]: http://duckietown.org/
[duckiebook]: http://book.duckietown.org/

<img src="http://duckietown.org/media/duckie2.png" width="120" height="120"/>

#### Continuous integration

If you do not know what "continuous integration" is, please see
 <a href="http://purl.org/dth/continuous-integration">this chapter</a> in the Duckiebook.

See [the continuous integration console](https://circleci.com/gh/duckietown/Software/).

Master is now a protected branch. [See docs here](https://github.com/blog/2051-protected-branches-and-required-status-checks).


<table>
<thead>
    <tr><td>Branch</td><td>tests</td><td></td></tr>
</thead>
<tbody>
    <tr>
        <td> <code>master</code> </td>
        <td>
            <a href="https://circleci.com/gh/duckietown/Software/tree/master">
                <img src='https://circleci.com/gh/duckietown/Software/tree/master.svg?style=shield'/></a>
        </td>
        <td>Merges to master are not allowed unless the tests pass. Master will always be green. </td>
    </tr>
    <tr>
        <td> <code>andrea-devel</code> </td>
        <td>
        <a href="https://circleci.com/gh/duckietown/Software/tree/andrea-devel">
        <img src='https://circleci.com/gh/duckietown/Software/tree/andrea-devel.svg?style=shield'/>
        </a>
        </td>
        <td> Andrea's reorganization/cleanup branch. </td>
    </tr>
    <tr>
        <td> <code>liam-devel</code> </td>
        <td>
        <a href="https://circleci.com/gh/duckietown/Software/tree/liam-devel">
        <img src='https://circleci.com/gh/duckietown/Software/tree/liam-devel.svg?style=shield'/>
        </a>
        </td>
        <td> Liam's branch. </td>
    </tr>
</tbody>
</table>
