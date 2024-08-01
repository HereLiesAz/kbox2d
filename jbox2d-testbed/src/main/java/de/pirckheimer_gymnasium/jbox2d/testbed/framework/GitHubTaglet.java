package de.pirckheimer_gymnasium.jbox2d.testbed.framework;

import com.sun.source.doctree.DocTree;
import jdk.javadoc.doclet.Taglet;

import javax.lang.model.element.Element;
import java.util.List;
import java.util.Set;

public class GitHubTaglet implements Taglet
{
    private static final String NAME = "box2d";

    @Override
    public String getName()
    {
        return NAME;
    }

    @Override
    public Set<Location> getAllowedLocations()
    {
        return Set.of(Location.METHOD, Location.TYPE);
    }

    @Override
    public boolean isInlineTag()
    {
        return false;
    }

    @Override
    public String toString(List<? extends DocTree> tags, Element element)
    {
        return "sb.toString()";
    }
}
